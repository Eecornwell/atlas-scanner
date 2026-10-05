#import "FeatureMatcher.h"
#import <opencv2/opencv.hpp>
#import <opencv2/features.hpp>

// Lowe's ratio test threshold — matches flutter-pixelmatching Constants.h
static const float kRatioThreshold = 0.75f;
// Minimum matches required to consider the result valid
static const int   kMinMatches     = 8;
// kNN k value
static const int   kKNN            = 2;
// Downsample ERP to this width before matching (speed vs quality trade-off)
// Full X5 ERP is 8000px wide — KAZE on that would be very slow on-device.
static const int   kMaxERPWidth    = 1024;

static cv::Mat uiImageToGrayMat(UIImage *image, int maxWidth) {
    CGImageRef cgImg = image.CGImage;
    size_t w = CGImageGetWidth(cgImg);
    size_t h = CGImageGetHeight(cgImg);

    // Render to grayscale bitmap
    cv::Mat gray((int)h, (int)w, CV_8UC1);
    CGColorSpaceRef cs = CGColorSpaceCreateDeviceGray();
    CGContextRef ctx = CGBitmapContextCreate(
        gray.data, w, h, 8, gray.step[0], cs,
        kCGImageAlphaNone | kCGBitmapByteOrderDefault
    );
    CGColorSpaceRelease(cs);
    CGContextDrawImage(ctx, CGRectMake(0, 0, w, h), cgImg);
    CGContextRelease(ctx);

    // Downsample if wider than maxWidth
    if (maxWidth > 0 && (int)w > maxWidth) {
        int newH = (int)((float)h * maxWidth / (float)w);
        cv::Mat small;
        cv::resize(gray, small, cv::Size(maxWidth, newH), 0, 0, cv::INTER_AREA);
        return small;
    }
    return gray;
}

@implementation FeatureMatcher {
    cv::Ptr<cv::Feature2D>         _detector;
    cv::Ptr<cv::DescriptorMatcher> _matcher;
    std::vector<cv::KeyPoint>      _refKeypoints;
    cv::Mat                        _refDescriptors;
    cv::Size                       _refSize;
    cv::Size                       _refOrig;
    NSInteger                      _matchCount;
    BOOL                           _useBFMatcher;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _matchCount = 0;
        _useBFMatcher = false;
        @try {
            _detector = cv::SIFT::create();
        } @catch (NSException *e) {
            NSLog(@"[FeatureMatcher] SIFT::create failed: %@", e);
        }
        if (!_detector) {
            NSLog(@"[FeatureMatcher] SIFT unavailable, using ORB");
            _detector = cv::ORB::create(2000);
            _useBFMatcher = true;
        }
        if (_useBFMatcher) {
            _matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        } else {
            _matcher = cv::DescriptorMatcher::create(
                cv::DescriptorMatcher::MatcherType::FLANNBASED
            );
        }
        NSLog(@"[FeatureMatcher] init: detector=%s matcher=%s",
              _useBFMatcher ? "ORB" : "SIFT",
              _useBFMatcher ? "BFMatcher" : "FLANN");
    }
    return self;
}

- (BOOL)setReference:(UIImage *)lidarERP {
    @try {
        _refOrig = cv::Size((int)lidarERP.size.width, (int)lidarERP.size.height);
        cv::Mat gray = uiImageToGrayMat(lidarERP, kMaxERPWidth);
        if (gray.empty()) {
            NSLog(@"[FeatureMatcher] setReference: gray image is empty");
            return false;
        }
        _refSize = gray.size();
        NSLog(@"[FeatureMatcher] setReference: gray %dx%d", _refSize.width, _refSize.height);

        _refKeypoints.clear();
        _refDescriptors.release();
        _detector->detectAndCompute(gray, cv::noArray(), _refKeypoints, _refDescriptors);
        NSLog(@"[FeatureMatcher] setReference: %d keypoints, desc rows=%d",
              (int)_refKeypoints.size(), _refDescriptors.rows);

        if (_refDescriptors.empty() || _refDescriptors.rows < kKNN) return false;

        _matcher->clear();
        _matcher->add(_refDescriptors);
        return YES;
    } @catch (NSException *e) {
        NSLog(@"[FeatureMatcher] setReference exception: %@", e);
        return false;
    }
}

- (nullable NSArray<NSValue *> *)match:(UIImage *)instaERP {
    @try {
        if (_refDescriptors.empty()) {
            NSLog(@"[FeatureMatcher] match: no reference descriptors");
            return nil;
        }

        cv::Size instaOrig((int)instaERP.size.width, (int)instaERP.size.height);
        cv::Mat gray = uiImageToGrayMat(instaERP, kMaxERPWidth);
        if (gray.empty()) {
            NSLog(@"[FeatureMatcher] match: gray image is empty");
            return nil;
        }
        cv::Size instaSize = gray.size();
        NSLog(@"[FeatureMatcher] match: query %dx%d", instaSize.width, instaSize.height);

        std::vector<cv::KeyPoint> queryKpts;
        cv::Mat queryDesc;
        _detector->detectAndCompute(gray, cv::noArray(), queryKpts, queryDesc);
        NSLog(@"[FeatureMatcher] match: %d query keypoints, desc rows=%d",
              (int)queryKpts.size(), queryDesc.rows);
        if (queryDesc.empty() || queryDesc.rows < kKNN) return nil;

        std::vector<std::vector<cv::DMatch>> knnMatches;
        try {
            _matcher->knnMatch(queryDesc, _refDescriptors, knnMatches, kKNN);
        } catch (const cv::Exception &e) {
            NSLog(@"[FeatureMatcher] knnMatch cv::Exception: %s", e.what());
            return nil;
        } catch (const std::exception &e) {
            NSLog(@"[FeatureMatcher] knnMatch std::exception: %s", e.what());
            return nil;
        } catch (...) {
            NSLog(@"[FeatureMatcher] knnMatch unknown exception");
            return nil;
        }
        NSLog(@"[FeatureMatcher] match: %d raw kNN matches", (int)knnMatches.size());

        NSMutableArray<NSValue *> *result = [NSMutableArray array];
        for (auto &m : knnMatches) {
            if (m.size() < 2) continue;
            if (m[0].distance >= kRatioThreshold * m[1].distance) continue;

            int qIdx = m[0].queryIdx;
            int tIdx = m[0].trainIdx;
            if (qIdx < 0 || qIdx >= (int)queryKpts.size()) continue;
            if (tIdx < 0 || tIdx >= (int)_refKeypoints.size()) continue;

            const cv::KeyPoint &qKpt = queryKpts[qIdx];
            const cv::KeyPoint &rKpt = _refKeypoints[tIdx];

            float scaleRef   = (float)_refOrig.width  / (float)_refSize.width;
            float scaleInsta = (float)instaOrig.width  / (float)instaSize.width;

            MatchedPair pair;
            pair.lidarPt = CGPointMake(rKpt.pt.x * scaleRef,   rKpt.pt.y * scaleRef);
            pair.instaPt = CGPointMake(qKpt.pt.x * scaleInsta, qKpt.pt.y * scaleInsta);
            [result addObject:[NSValue valueWithBytes:&pair objCType:@encode(MatchedPair)]];
        }

        _matchCount = (NSInteger)result.count;
        NSLog(@"[FeatureMatcher] match: %ld good matches (min %d required)",
              (long)_matchCount, kMinMatches);
        return _matchCount >= kMinMatches ? result : nil;
    } @catch (NSException *e) {
        NSLog(@"[FeatureMatcher] match exception: %@", e);
        return nil;
    }
}

- (NSInteger)matchCount {
    return _matchCount;
}

@end
