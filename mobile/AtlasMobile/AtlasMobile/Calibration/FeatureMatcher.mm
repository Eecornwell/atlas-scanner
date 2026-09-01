#import "FeatureMatcher.h"
#import <opencv2/opencv.hpp>
#import <opencv2/features2d.hpp>

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
    cv::Ptr<cv::KAZE>              _detector;
    cv::Ptr<cv::DescriptorMatcher> _matcher;
    std::vector<cv::KeyPoint>      _refKeypoints;
    cv::Mat                        _refDescriptors;
    cv::Size                       _refSize;   // size of downsampled reference
    cv::Size                       _refOrig;   // original reference image size
    NSInteger                      _matchCount;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        // KAZE: good for ERP images — handles the smooth gradients and
        // non-planar distortion better than SIFT on equirectangular content.
        _detector = cv::KAZE::create();
        _matcher  = cv::DescriptorMatcher::create(
            cv::DescriptorMatcher::MatcherType::FLANNBASED
        );
        _matchCount = 0;
    }
    return self;
}

- (BOOL)setReference:(UIImage *)lidarERP {
    _refOrig = cv::Size((int)lidarERP.size.width, (int)lidarERP.size.height);
    cv::Mat gray = uiImageToGrayMat(lidarERP, kMaxERPWidth);
    if (gray.empty()) return NO;
    _refSize = gray.size();

    _refKeypoints.clear();
    _refDescriptors.release();
    _detector->detectAndCompute(gray, cv::noArray(), _refKeypoints, _refDescriptors);

    if (_refDescriptors.empty() || _refDescriptors.rows < kKNN) return NO;

    _matcher->clear();
    _matcher->add(_refDescriptors);
    return YES;
}

- (nullable NSArray<NSValue *> *)match:(UIImage *)instaERP {
    if (_refDescriptors.empty()) return nil;

    cv::Size instaOrig((int)instaERP.size.width, (int)instaERP.size.height);
    cv::Mat gray = uiImageToGrayMat(instaERP, kMaxERPWidth);
    if (gray.empty()) return nil;
    cv::Size instaSize = gray.size();

    std::vector<cv::KeyPoint> queryKpts;
    cv::Mat queryDesc;
    _detector->detectAndCompute(gray, cv::noArray(), queryKpts, queryDesc);
    if (queryDesc.empty() || queryDesc.rows < kKNN) return nil;

    // kNN match
    std::vector<std::vector<cv::DMatch>> knnMatches;
    try {
        _matcher->knnMatch(queryDesc, _refDescriptors, knnMatches, kKNN);
    } catch (...) {
        return nil;
    }

    // Lowe's ratio test
    NSMutableArray<NSValue *> *result = [NSMutableArray array];
    for (auto &m : knnMatches) {
        if (m.size() < 2) continue;
        if (m[0].distance >= kRatioThreshold * m[1].distance) continue;

        const cv::KeyPoint &qKpt = queryKpts[m[0].queryIdx];
        const cv::KeyPoint &rKpt = _refKeypoints[m[0].trainIdx];

        // Scale coordinates back to original image dimensions
        float scaleRef   = (float)_refOrig.width  / (float)_refSize.width;
        float scaleInsta = (float)instaOrig.width  / (float)instaSize.width;

        MatchedPair pair;
        pair.lidarPt = CGPointMake(rKpt.pt.x * scaleRef,   rKpt.pt.y * scaleRef);
        pair.instaPt = CGPointMake(qKpt.pt.x * scaleInsta, qKpt.pt.y * scaleInsta);
        [result addObject:[NSValue valueWithBytes:&pair objCType:@encode(MatchedPair)]];
    }

    _matchCount = (NSInteger)result.count;
    return _matchCount >= kMinMatches ? result : nil;
}

- (NSInteger)matchCount {
    return _matchCount;
}

@end
