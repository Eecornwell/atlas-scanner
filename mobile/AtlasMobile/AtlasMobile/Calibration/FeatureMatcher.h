#pragma once
#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

/// A matched pair of pixel coordinates: one from the LiDAR intensity ERP,
/// one from the Insta360 ERP.
typedef struct {
    CGPoint lidarPt;    // pixel in LiDAR intensity ERP
    CGPoint instaPt;    // pixel in Insta360 ERP
} MatchedPair;

/// Wraps OpenCV KAZE + FLANN kNN matching with Lowe's ratio test.
/// Returns matched pixel coordinate pairs between a LiDAR intensity ERP
/// and an Insta360 ERP image.
///
/// Mirrors flutter-pixelmatching's ImageCompare.cpp algorithm:
///   - Detector:  KAZE (rotation/scale invariant, good on ERP crops)
///   - Matcher:   FLANN kNN (k=2) with Lowe's ratio test (threshold 0.75)
///   - No homography — we return raw matched coordinates for use in the
///     6-DOF reprojection cost function.
@interface FeatureMatcher : NSObject

/// Detects and stores keypoints + descriptors for the reference (LiDAR ERP).
/// Must be called before match().
- (BOOL)setReference:(UIImage *)lidarERP;

/// Matches the query (Insta360 ERP) against the stored reference.
/// Returns an NSArray of NSValue-wrapped MatchedPair structs, or nil on failure.
- (nullable NSArray<NSValue *> *)match:(UIImage *)instaERP;

/// Number of matches from the last match() call.
@property (nonatomic, readonly) NSInteger matchCount;

@end
