#import "SfMTriangulator.h"
#import <UIKit/UIKit.h>
#import <opencv2/opencv.hpp>
#import <opencv2/features2d.hpp>

static const float kRatioThreshold = 0.75f;
static const int   kMinTrackLen    = 2;
static const float kMaxReproj      = 4.0f;   // pixels
static const int   kMaxImageWidth  = 1024;
static const int   kMatchWindow    = 2;       // match i→i+1 and i→i+2

static int cvErrorHandler(int status, const char* func,
                          const char* msg, const char* file,
                          int line, void*) {
    NSLog(@"[OpenCV] error %d in %s: %s (%s:%d)",
          status, func, msg, file, line);
    return 0;
}

// -----------------------------------------------------------------
// Internal types
// -----------------------------------------------------------------

struct Observation {
    int imageIdx;
    int keypointIdx;
    float px, py; // pixel coords in full-res image
};

struct Track3D {
    cv::Point3f pos;
    cv::Vec3b   color;
    float       error;
    std::vector<Observation> obs;
};

// Union-Find for track merging
struct UnionFind {
    std::unordered_map<int64_t, int64_t> parent;

    int64_t find(int64_t x) {
        if (parent.find(x) == parent.end()) parent[x] = x;
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }
    void unite(int64_t a, int64_t b) {
        a = find(a); b = find(b);
        if (a != b) parent[b] = a;
    }
};

// -----------------------------------------------------------------

static cv::Mat loadGray(NSString *path, float &scale) {
    UIImage *img = [UIImage imageWithContentsOfFile:path];
    if (!img) return {};
    CGImageRef cg = img.CGImage;
    int w = (int)CGImageGetWidth(cg);
    int h = (int)CGImageGetHeight(cg);

    cv::Mat gray(h, w, CV_8UC1);
    CGColorSpaceRef cs = CGColorSpaceCreateDeviceGray();
    CGContextRef ctx = CGBitmapContextCreate(
        gray.data, w, h, 8, gray.step[0], cs,
        kCGImageAlphaNone | kCGBitmapByteOrderDefault);
    CGColorSpaceRelease(cs);
    if (!ctx) {
        NSLog(@"[SfMTriangulator] CGBitmapContext FAILED for %dx%d", w, h);
        scale = 1.0f;
        return {};
    }
    CGContextDrawImage(ctx, CGRectMake(0, 0, w, h), cg);
    CGContextRelease(ctx);

    if (w > kMaxImageWidth) {
        float s = (float)kMaxImageWidth / (float)w;
        int newH = (int)(h * s);
        cv::Mat small;
        cv::resize(gray, small, cv::Size(kMaxImageWidth, newH),
                   0, 0, cv::INTER_AREA);
        scale = 1.0f / s;
        return small;
    }
    scale = 1.0f;
    return gray;
}

// Pre-loaded image pixel cache — downsampled to kMaxImageWidth
struct ImagePixels {
    std::vector<uint8_t> rgba;
    int w = 0, h = 0;
    int origW = 0, origH = 0;
    bool valid = false;
};

static ImagePixels loadPixels(NSString *path) {
    @autoreleasepool {
        UIImage *img = [UIImage imageWithContentsOfFile:path];
        if (!img || !img.CGImage) return {};
        CGImageRef cg = img.CGImage;
        int origW = (int)CGImageGetWidth(cg);
        int origH = (int)CGImageGetHeight(cg);
        if (origW <= 0 || origH <= 0) return {};
        int w = origW, h = origH;
        if (w > kMaxImageWidth) {
            float s = (float)kMaxImageWidth / (float)w;
            h = (int)(h * s);
            w = kMaxImageWidth;
        }
        ImagePixels px;
        px.w = w; px.h = h;
        px.origW = origW; px.origH = origH;
        px.rgba.resize(w * h * 4);
        CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
        CGContextRef ctx = CGBitmapContextCreate(
            px.rgba.data(), w, h, 8, w * 4, cs,
            kCGImageAlphaNoneSkipLast | kCGBitmapByteOrderDefault);
        CGColorSpaceRelease(cs);
        if (!ctx) return {};
        CGContextDrawImage(ctx, CGRectMake(0, 0, w, h), cg);
        CGContextRelease(ctx);
        px.valid = true;
        return px;
    }
}

static cv::Vec3b sampleCached(const ImagePixels &px, float x, float y) {
    if (!px.valid) return {128, 128, 128};
    float sx = (float)px.w / (float)px.origW;
    float sy = (float)px.h / (float)px.origH;
    int ix = std::clamp((int)(x * sx), 0, px.w - 1);
    int iy = std::clamp((int)(y * sy), 0, px.h - 1);
    int off = (iy * px.w + ix) * 4;
    return {px.rgba[off], px.rgba[off+1], px.rgba[off+2]};
}

static cv::Mat projDataToMat(NSData *data) {
    if (data.length < 12 * sizeof(float)) return {};
    const float *f = (const float *)data.bytes;
    cv::Mat P(3, 4, CV_64F);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 4; c++)
            P.at<double>(r, c) = (double)f[r * 4 + c];
    return P;
}

static float reprojectError(
    const cv::Point3f &pt3D,
    const cv::Mat &P,
    float obs_x, float obs_y
) {
    double X = pt3D.x, Y = pt3D.y, Z = pt3D.z;
    double px = P.at<double>(0,0)*X + P.at<double>(0,1)*Y
              + P.at<double>(0,2)*Z + P.at<double>(0,3);
    double py = P.at<double>(1,0)*X + P.at<double>(1,1)*Y
              + P.at<double>(1,2)*Z + P.at<double>(1,3);
    double pw = P.at<double>(2,0)*X + P.at<double>(2,1)*Y
              + P.at<double>(2,2)*Z + P.at<double>(2,3);
    if (std::abs(pw) < 1e-8) return 1e6;
    float rx = (float)(px / pw);
    float ry = (float)(py / pw);
    float dx = rx - obs_x;
    float dy = ry - obs_y;
    return std::sqrt(dx * dx + dy * dy);
}

static bool positiveDepth(
    const cv::Point3f &pt3D, const cv::Mat &P
) {
    double X = pt3D.x, Y = pt3D.y, Z = pt3D.z;
    double d = P.at<double>(2,0)*X + P.at<double>(2,1)*Y
             + P.at<double>(2,2)*Z + P.at<double>(2,3);
    return d > 0;
}

// -----------------------------------------------------------------
// Implementation
// -----------------------------------------------------------------

@implementation SfMTriangulator {
    std::vector<Track3D> _tracks;
}

- (NSInteger)pointCount {
    return (NSInteger)_tracks.size();
}

- (NSInteger)triangulateImages:(NSArray<NSString *> *)imagePaths
            projectionMatrices:(NSArray<NSData *> *)projMatrices
{
    _tracks.clear();
    NSInteger N = imagePaths.count;
    if (N < 2 || (NSInteger)projMatrices.count != N) return -1;

    NSLog(@"[SfMTriangulator] starting with %ld images", (long)N);

    cv::redirectError(cvErrorHandler);

  try {

    // 1. Create ORB feature detector (always in features2d).
    //    SIFT causes signal crashes on some iOS OpenCV builds
    //    even inside try/catch, so we use ORB exclusively.
    NSLog(@"[SfMTriangulator] creating ORB detector...");
    cv::Ptr<cv::Feature2D> detector;
    try {
        detector = cv::ORB::create(4000);
    } catch (const std::exception &e) {
        NSLog(@"[SfMTriangulator] ORB failed: %s", e.what());
        return -1;
    } catch (...) {
        NSLog(@"[SfMTriangulator] ORB failed (unknown)");
        return -1;
    }
    if (!detector) {
        NSLog(@"[SfMTriangulator] ORB::create returned null");
        return -1;
    }
    NSLog(@"[SfMTriangulator] ORB detector created OK");

    struct ImageFeatures {
        std::vector<cv::KeyPoint> kpts;
        cv::Mat desc;
        float scale;
    };
    std::vector<ImageFeatures> features(N);

    // TODO: ORB detectAndCompute crashes on this OpenCV build (ARM64).
    // Needs OpenCV rebuild or alternative detector (AKAZE).
    NSLog(@"[SfMTriangulator] feature detection disabled — ORB crash on ARM64");
    return 0;

    for (NSInteger i = 0; i < N; i++) { @autoreleasepool {
        NSLog(@"[SfMTriangulator] loading image %ld...", (long)i);

        float scale = 1.0f;
        cv::Mat gray = loadGray(imagePaths[i], scale);
        if (gray.empty()) {
            NSLog(@"[SfMTriangulator] failed to load image %ld", (long)i);
            continue;
        }
        NSLog(@"[SfMTriangulator] detectAndCompute image %ld (%dx%d)...",
              (long)i, gray.cols, gray.rows);
        try {
            detector->detectAndCompute(
                gray, cv::noArray(),
                features[i].kpts, features[i].desc);
        } catch (const std::exception &e) {
            NSLog(@"[SfMTriangulator] SIFT detect failed image %ld: %s",
                  (long)i, e.what());
            continue;
        } catch (...) {
            NSLog(@"[SfMTriangulator] SIFT detect failed image %ld",
                  (long)i);
            continue;
        }
        features[i].scale = scale;
        NSLog(@"[SfMTriangulator] image %ld: %d keypoints",
              (long)i, (int)features[i].kpts.size());
    } }

    // 2. Parse projection matrices
    std::vector<cv::Mat> P(N);
    for (NSInteger i = 0; i < N; i++) {
        P[i] = projDataToMat(projMatrices[i]);
        if (P[i].empty()) {
            NSLog(@"[SfMTriangulator] bad projection matrix %ld", (long)i);
            return -1;
        }
    }

    // 3. Sequential matching + union-find for tracks
    NSLog(@"[SfMTriangulator] starting matching...");
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    const int kMaxKpts = 10000;

    UnionFind uf;
    // (imageIdx, kptIdx) → full-res pixel coords
    std::unordered_map<int64_t, std::pair<float,float>> idToPixel;

    auto makeId = [&](int img, int kpt) -> int64_t {
        return (int64_t)img * kMaxKpts + kpt;
    };

    int totalMatches = 0;

    for (NSInteger i = 0; i < N; i++) {
        int jMax = std::min(i + kMatchWindow, N - 1);
        for (NSInteger j = i + 1; j <= jMax; j++) {
            if (features[i].desc.empty() || features[j].desc.empty())
                continue;
            if (features[i].desc.rows < 2 || features[j].desc.rows < 2)
                continue;

            std::vector<std::vector<cv::DMatch>> knnMatches;
            try {
                matcher.knnMatch(
                    features[i].desc, features[j].desc,
                    knnMatches, 2);
            } catch (...) {
                NSLog(@"[SfMTriangulator] match %ld→%ld failed",
                      (long)i, (long)j);
                continue;
            }

            int pairMatches = 0;
            for (auto &m : knnMatches) {
                if (m.size() < 2) continue;
                if (m[0].distance >= kRatioThreshold * m[1].distance)
                    continue;

                int qi = m[0].queryIdx;
                int ti = m[0].trainIdx;

                float px1 = features[i].kpts[qi].pt.x * features[i].scale;
                float py1 = features[i].kpts[qi].pt.y * features[i].scale;
                float px2 = features[j].kpts[ti].pt.x * features[j].scale;
                float py2 = features[j].kpts[ti].pt.y * features[j].scale;

                int64_t id1 = makeId((int)i, qi);
                int64_t id2 = makeId((int)j, ti);
                uf.unite(id1, id2);
                idToPixel[id1] = {px1, py1};
                idToPixel[id2] = {px2, py2};
                pairMatches++;
            }
            totalMatches += pairMatches;
            NSLog(@"[SfMTriangulator] match %ld→%ld: %d good matches",
                  (long)i, (long)j, pairMatches);
        }
    }

    NSLog(@"[SfMTriangulator] total good matches: %d", totalMatches);

    // 4. Group into tracks by union-find root
    std::unordered_map<int64_t, std::vector<std::pair<int,int>>> trackGroups;
    for (auto &[id, px] : idToPixel) {
        int64_t root = uf.find(id);
        int imgIdx = (int)(id / kMaxKpts);
        int kptIdx = (int)(id % kMaxKpts);
        trackGroups[root].push_back({imgIdx, kptIdx});
    }

    NSLog(@"[SfMTriangulator] %d raw tracks", (int)trackGroups.size());

    // 4b. Pre-cache image pixel data for color sampling
    std::vector<ImagePixels> pixelCache(N);
    for (NSInteger i = 0; i < N; i++) {
        pixelCache[i] = loadPixels(imagePaths[i]);
    }
    NSLog(@"[SfMTriangulator] pixel cache loaded for %ld images", (long)N);

    // 5. Triangulate each track
    NSLog(@"[SfMTriangulator] triangulating %d tracks...",
          (int)trackGroups.size());
    int accepted = 0, rejected = 0;

    for (auto &[root, members] : trackGroups) {
        if ((int)members.size() < kMinTrackLen) continue;

        // Deduplicate: keep one observation per image
        std::unordered_map<int, int> imgToKpt;
        for (auto &[img, kpt] : members) {
            imgToKpt[img] = kpt; // last wins
        }
        if ((int)imgToKpt.size() < kMinTrackLen) continue;

        // Find the pair with widest baseline for triangulation
        std::vector<int> imgIndices;
        for (auto &[img, kpt] : imgToKpt) imgIndices.push_back(img);
        std::sort(imgIndices.begin(), imgIndices.end());

        int idxA = imgIndices.front();
        int idxB = imgIndices.back();
        int kptA = imgToKpt[idxA];
        int kptB = imgToKpt[idxB];

        int64_t idA = makeId(idxA, kptA);
        int64_t idB = makeId(idxB, kptB);
        auto pxA = idToPixel[idA];
        auto pxB = idToPixel[idB];

        // DLT triangulation (avoids calib3d dependency)
        double x1 = pxA.first,  y1 = pxA.second;
        double x2 = pxB.first,  y2 = pxB.second;
        cv::Mat A(4, 4, CV_64F);
        for (int c = 0; c < 4; c++) {
            A.at<double>(0,c) = x1*P[idxA].at<double>(2,c) - P[idxA].at<double>(0,c);
            A.at<double>(1,c) = y1*P[idxA].at<double>(2,c) - P[idxA].at<double>(1,c);
            A.at<double>(2,c) = x2*P[idxB].at<double>(2,c) - P[idxB].at<double>(0,c);
            A.at<double>(3,c) = y2*P[idxB].at<double>(2,c) - P[idxB].at<double>(1,c);
        }
        cv::Mat w_svd, u_svd, vt_svd;
        cv::SVD::compute(A, w_svd, u_svd, vt_svd, cv::SVD::FULL_UV);
        double hw = vt_svd.at<double>(3, 3);
        if (std::abs(hw) < 1e-8) { rejected++; continue; }

        cv::Point3f pt3D(
            (float)(vt_svd.at<double>(3, 0) / hw),
            (float)(vt_svd.at<double>(3, 1) / hw),
            (float)(vt_svd.at<double>(3, 2) / hw)
        );

        // Validate: positive depth in both triangulation views
        if (!positiveDepth(pt3D, P[idxA]) ||
            !positiveDepth(pt3D, P[idxB])) {
            rejected++;
            continue;
        }

        // Compute mean reprojection error across all observations
        float totalError = 0;
        int validObs = 0;
        std::vector<Observation> observations;

        for (int imgI : imgIndices) {
            int kptI = imgToKpt[imgI];
            int64_t id = makeId(imgI, kptI);
            auto px = idToPixel[id];

            float err = reprojectError(pt3D, P[imgI], px.first, px.second);
            if (err > kMaxReproj) continue;

            totalError += err;
            validObs++;
            observations.push_back({imgI, kptI, px.first, px.second});
        }

        if (validObs < kMinTrackLen) { rejected++; continue; }

        float meanError = totalError / validObs;

        // Sample color from the first observing image
        cv::Vec3b color = sampleCached(
            pixelCache[observations[0].imageIdx],
            observations[0].px, observations[0].py);

        _tracks.push_back({pt3D, color, meanError, observations});
        accepted++;
    }

    NSLog(@"[SfMTriangulator] triangulated: %d accepted, %d rejected",
          accepted, rejected);
    return (NSInteger)accepted;

  } catch (const std::exception &e) {
    NSLog(@"[SfMTriangulator] FATAL: %s", e.what());
    return -1;
  } catch (...) {
    NSLog(@"[SfMTriangulator] FATAL: unknown C++ exception");
    return -1;
  }
}

- (void)point:(NSInteger)idx
            x:(float *)x y:(float *)y z:(float *)z
            r:(uint8_t *)r g:(uint8_t *)g b:(uint8_t *)b
        error:(float *)error
{
    if (idx < 0 || idx >= (NSInteger)_tracks.size()) return;
    const auto &t = _tracks[idx];
    *x = t.pos.x; *y = t.pos.y; *z = t.pos.z;
    *r = t.color[0]; *g = t.color[1]; *b = t.color[2];
    *error = t.error;
}

- (NSInteger)trackLengthForPoint:(NSInteger)idx {
    if (idx < 0 || idx >= (NSInteger)_tracks.size()) return 0;
    return (NSInteger)_tracks[idx].obs.size();
}

- (void)observation:(NSInteger)obsIdx
           forPoint:(NSInteger)ptIdx
         imageIndex:(int32_t *)imageIndex
                 px:(float *)px
                 py:(float *)py
{
    if (ptIdx < 0 || ptIdx >= (NSInteger)_tracks.size()) return;
    const auto &obs = _tracks[ptIdx].obs;
    if (obsIdx < 0 || obsIdx >= (NSInteger)obs.size()) return;
    *imageIndex = (int32_t)obs[obsIdx].imageIdx;
    *px = obs[obsIdx].px;
    *py = obs[obsIdx].py;
}

- (NSInteger)safeTriangulateImages:(NSArray<NSString *> *)imagePaths
                projectionMatrices:(NSArray<NSData *> *)projMatrices
                             error:(NSError **)error
{
    NSLog(@"[SfMTriangulator] safeTriangulate entry — %lu images",
          (unsigned long)imagePaths.count);
    @try {
        NSInteger result = [self triangulateImages:imagePaths
                            projectionMatrices:projMatrices];
        NSLog(@"[SfMTriangulator] safeTriangulate done — %ld pts",
              (long)result);
        return result;
    } @catch (NSException *exception) {
        NSLog(@"[SfMTriangulator] ObjC exception: %@ — %@",
              exception.name, exception.reason);
        if (error) {
            *error = [NSError errorWithDomain:@"SfMTriangulator"
                                         code:-1
                                     userInfo:@{
                NSLocalizedDescriptionKey:
                    [NSString stringWithFormat:
                        @"Triangulation failed: %@", exception.reason]
            }];
        }
        return -1;
    }
}

@end
