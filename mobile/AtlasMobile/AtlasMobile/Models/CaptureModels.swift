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
    /// ARKit tracking state when the frame was taken: "normal",
    /// "limited:<reason>" or "notAvailable".
    let trackingState: String
}

struct Insta360CaptureResult {
    let cameraId: String
    let scanIndex: Int
    let arkitTimestamp: Double
    let commandTimestamp: Double
    let callbackTimestamp: Double
    let mediaIdentifier: String
}

struct Insta360DownloadResult {
    let cameraId: String
    let scanIndex: Int
    let localURL: URL
    let mediaType: MediaType
    let exifTimestamp: Double?

    enum MediaType {
        case equirectangular
        case dualFisheye
        case raw
    }
}

struct ClockOffset {
    let offsetMs: Double
    let uncertaintyMs: Double
    let sampleCount: Int
    let preSetSample: ClockSyncSample?
    let samples: [ClockSyncSample]
    let wallClockSec: Double
    let arkitTimeSec: Double
}

struct ClockSyncSample {
    let phoneSendMs: Double
    let phoneRecvMs: Double
    let cameraMs: Double
    let offsetMs: Double
    var roundTripMs: Double { phoneRecvMs - phoneSendMs }
}

struct PoseEntry: Codable {
    let timestamp: Double
    let transform: [[Float]]
}

struct TrajectoryData: Codable {
    let poses: [PoseEntry]
}
