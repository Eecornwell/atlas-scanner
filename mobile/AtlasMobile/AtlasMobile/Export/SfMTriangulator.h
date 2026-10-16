#pragma once
#import <Foundation/Foundation.h>

/// On-device SfM triangulation using OpenCV SIFT.
///
/// Extracts SIFT features from iPhone images, matches sequential
/// pairs, builds multi-view tracks, and triangulates 3D points
/// using known camera poses from ARKit.
@interface SfMTriangulator : NSObject

/// Run feature extraction, matching, and triangulation.
/// @param imagePaths  Ordered JPEG paths (portrait iPhone images)
/// @param projMatrices  Parallel array of NSData, each 12 floats
///   (3×4 row-major projection matrix P = K·[R|t], world-to-camera)
/// @return Number of triangulated 3D points, or -1 on failure
- (NSInteger)triangulateImages:(NSArray<NSString *> *)imagePaths
            projectionMatrices:(NSArray<NSData *> *)projMatrices;

/// Safe wrapper: catches ObjC/C++ exceptions, returns -1 + error.
- (NSInteger)safeTriangulateImages:(NSArray<NSString *> *)imagePaths
                projectionMatrices:(NSArray<NSData *> *)projMatrices
                             error:(NSError **)error;

/// Number of triangulated 3D points after triangulateImages:.
@property (nonatomic, readonly) NSInteger pointCount;

/// Get the 3D position and color of a triangulated point.
- (void)point:(NSInteger)idx
            x:(float *)x y:(float *)y z:(float *)z
            r:(uint8_t *)r g:(uint8_t *)g b:(uint8_t *)b
        error:(float *)error;

/// Number of image observations for point at idx.
- (NSInteger)trackLengthForPoint:(NSInteger)idx;

/// Get one observation (image index + pixel coords) for a point.
- (void)observation:(NSInteger)obsIdx
           forPoint:(NSInteger)ptIdx
         imageIndex:(int32_t *)imageIndex
                 px:(float *)px
                 py:(float *)py;

@end
