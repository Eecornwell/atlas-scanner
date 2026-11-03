#pragma once
#import <Foundation/Foundation.h>

/// OpenCV helpers for on-device rig calibration (RigCalibrator.swift):
/// SIFT features and matching (RigSIFTFeatures), DLT triangulation of
/// iPhone tracks and PnP for X5 tiles.
///
/// The OpenCV crashes in this code (SIFT, ORB, SVD) came from linking:
/// part of OpenCV was taken from the Insta360 SDK's OpenCV 4.7 (exported
/// by INSCoreMedia.framework), whose cv::Mat layout differs from the
/// OpenCV 5 headers. project.yml now lists opencv2 first; see
/// openCVLinkedFromApp.
NS_ASSUME_NONNULL_BEGIN

/// OpenCV SIFT keypoints and descriptors per image ID, kept until this
/// object is released or reset.
@interface RigSIFTFeatures : NSObject

/// @param maxFeatures  Strongest features kept per image (0 = all).
- (instancetype)initWithMaxFeatures:(int)maxFeatures;

/// Detects SIFT features in a width×height 8-bit grayscale buffer (rows
/// packed, `width` bytes each) and stores them under `imageId`.
/// @return The feature count, or -1 on failure.
- (int)addGray:(const uint8_t *)gray
         width:(int)width
        height:(int)height
       imageId:(int)imageId
    NS_SWIFT_NAME(add(gray:width:height:imageId:));

/// Brute-force L2 matches from query to train that pass Lowe's ratio
/// test, as packed floats: query x, query y, train x, train y per match.
/// @return nil when either image has no features or nothing matches.
- (nullable NSData *)matchQuery:(int)queryId
                          train:(int)trainId
                          ratio:(float)ratio
    NS_SWIFT_NAME(match(query:train:ratio:));

/// Frees all stored features.
- (void)reset;

@end

@interface RigCalibratorBridge : NSObject

/// DLT triangulation from observations in multiple views.
/// @param observations  Each sub-array is [imageId, pixelX, pixelY].
/// @param projMats  imageId → NSData of 12 doubles (3×4 row-major P = K·[R|t]).
/// @param maxReproj  Reject observations with reprojection error above this.
/// @return [x, y, z, meanReprojError, validObsCount] or nil on failure.
+ (nullable NSArray<NSNumber *> *)
    triangulatePoint:(NSArray<NSArray<NSNumber *> *> *)observations
         projMatrices:(NSDictionary<NSNumber *, NSData *> *)projMats
            maxReproj:(float)maxReproj
          minObserved:(int)minObserved;

/// PnP RANSAC solve for camera pose from 3D-2D correspondences.
/// @return Dictionary with keys "rvec" (3 doubles), "tvec" (3 doubles),
///   "R" (9 doubles row-major), "inlierCount", or nil on failure.
+ (nullable NSDictionary *)
    solvePnPRansac:(NSArray<NSArray<NSNumber *> *> *)points3D
          points2D:(NSArray<NSArray<NSNumber *> *> *)points2D
                fx:(double)fx fy:(double)fy
                cx:(double)cx cy:(double)cy
        minInliers:(int)minInliers;

// MARK: - OpenCV linking check

/// Which binary each OpenCV function used here is linked from, e.g.
/// "cv::Mat::setTo AtlasMobile; cv::NAryMatIterator::init INSCoreMedia".
+ (NSString *)openCVBindingReport NS_SWIFT_NAME(openCVBindingReport());

/// YES when all the OpenCV functions in openCVBindingReport come from the
/// app binary, i.e. from the OpenCV 5 its headers describe. A function
/// taken from INSCoreMedia's OpenCV 4.7 reads cv::Mat at the wrong offsets.
+ (BOOL)openCVLinkedFromApp NS_SWIFT_NAME(openCVLinkedFromApp());

// MARK: - Crash diagnostics (export trace)

/// Starts writing crash details to `fd` (the open export trace): the
/// signal, faulting address, registers and a stack walk; uncaught C++ and
/// Objective-C exceptions; exit() calls. Handlers installed earlier are
/// kept and run afterwards. Returns lines describing the device, the app
/// image and the handlers that were already installed.
+ (NSString *)installCrashCaptureWithFileDescriptor:(int)fd
    NS_SWIFT_NAME(installCrashCapture(fileDescriptor:));

/// Stops writing crash details and puts the earlier handlers back.
+ (void)stopCrashCapture NS_SWIFT_NAME(stopCrashCapture());

/// Gives the calling thread its own signal stack, so a crash caused by a
/// stack overflow on this thread can still be written to the trace.
+ (void)useAlternateSignalStackOnCurrentThread
    NS_SWIFT_NAME(useAlternateSignalStackOnCurrentThread());

@end

NS_ASSUME_NONNULL_END
