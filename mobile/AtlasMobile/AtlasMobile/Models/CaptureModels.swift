import Foundation
import simd
import CoreVideo

struct ARKitFrameData {
    let timestamp: Double
    let pose: simd_float4x4
    let intrinsics: simd_float3x3
    let imageResolution: CGSize
    let capturedImage: CVPixelBuffer
    let depthMap: CVPixelBuffer
    let confidenceMap: CVPixelBuffer
    let smoothedDepthMap: CVPixelBuffer?
}

struct Insta360CaptureResult {
    let cameraId: String
    let scanIndex: Int
    let arkitTimestamp: Double
    let insta360Timestamp: Double
    let mediaIdentifier: String
}

struct Insta360DownloadResult {
    let cameraId: String
    let scanIndex: Int
    let localURL: URL
    let mediaType: MediaType

    enum MediaType {
        case equirectangular
        case dualFisheye
        case raw
    }
}

struct ClockOffset {
    let offsetMs: Double
    let sampleCount: Int
    let stdDevMs: Double
}

struct PoseEntry: Codable {
    let timestamp: Double
    let transform: [[Float]]
}

struct TrajectoryData: Codable {
    let poses: [PoseEntry]
}
