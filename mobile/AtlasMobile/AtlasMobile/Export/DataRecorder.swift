import Foundation
import CoreVideo
import simd

/// Records capture data to disk following the atlas-mobile directory structure.
final class DataRecorder {
    let sessionDirectory: URL

    private let iphoneDir: URL
    private let encoder = JSONEncoder()

    init(sessionDirectory: URL) {
        self.sessionDirectory = sessionDirectory
        self.iphoneDir = sessionDirectory.appendingPathComponent("raw/iphone")
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        createDirectoryStructure()
    }

    func saveScan(
        scanIndex: Int,
        arkitFrame: ARKitFrameData,
        insta360Results: [Insta360CaptureResult]
    ) async {
        let scanName = String(format: "scan_%03d", scanIndex)
        let scanDir = iphoneDir.appendingPathComponent(scanName)
        try? FileManager.default.createDirectory(at: scanDir, withIntermediateDirectories: true)

        // Save iPhone RGB
        await savePixelBufferAsJPEG(arkitFrame.capturedImage, to: scanDir.appendingPathComponent("rgb.jpg"))

        // Save depth map (Float32 binary)
        saveDepthMap(arkitFrame.depthMap, to: scanDir.appendingPathComponent("depth.bin"))

        // Save confidence map (UInt8 binary)
        saveConfidenceMap(arkitFrame.confidenceMap, to: scanDir.appendingPathComponent("confidence.bin"))

        // Save smoothed depth if available
        if let smoothed = arkitFrame.smoothedDepthMap {
            saveDepthMap(smoothed, to: scanDir.appendingPathComponent("depth_smoothed.bin"))
        }

        // Save pose JSON
        let poseData = ScanPose(
            timestamp: arkitFrame.timestamp,
            transformMatrix: matrixToArray(arkitFrame.pose),
            intrinsics: intrinsicsToArray(arkitFrame.intrinsics),
            imageWidth: Int(arkitFrame.imageResolution.width),
            imageHeight: Int(arkitFrame.imageResolution.height),
            depthWidth: CVPixelBufferGetWidth(arkitFrame.depthMap),
            depthHeight: CVPixelBufferGetHeight(arkitFrame.depthMap),
            insta360Captures: insta360Results.map { result in
                Insta360CaptureRef(
                    cameraId: result.cameraId,
                    timestamp: result.insta360Timestamp,
                    clockOffsetMs: result.arkitTimestamp - result.insta360Timestamp
                )
            }
        )
        if let poseJSON = try? encoder.encode(poseData) {
            try? poseJSON.write(to: scanDir.appendingPathComponent("pose.json"))
        }
    }

    func saveDownloadedMedia(_ downloads: [Insta360DownloadResult]) async {
        for download in downloads {
            let cameraDir = sessionDirectory
                .appendingPathComponent("raw")
                .appendingPathComponent(download.cameraId)
            try? FileManager.default.createDirectory(at: cameraDir, withIntermediateDirectories: true)

            let destName = String(format: "scan_%03d.jpg", download.scanIndex)
            let destURL = cameraDir.appendingPathComponent(destName)
            try? FileManager.default.copyItem(at: download.localURL, to: destURL)
        }
    }

    func saveTrajectory(_ trajectory: TrajectoryData?) async {
        guard let trajectory else { return }
        let url = sessionDirectory.appendingPathComponent("raw/trajectory.json")
        if let data = try? encoder.encode(trajectory) {
            try? data.write(to: url)
        }
    }

    // MARK: - Private

    private func createDirectoryStructure() {
        let dirs = [
            "raw/iphone",
            "colmap/images",
            "colmap/masks",
            "colmap/init_sparse/0",
            "colmap/sparse/0",
            "colmap/depth_images",
            "masks",
            "calibration",
            "enhanced/depth_dense",
            "enhanced/normals"
        ]
        for dir in dirs {
            try? FileManager.default.createDirectory(
                at: sessionDirectory.appendingPathComponent(dir),
                withIntermediateDirectories: true
            )
        }
    }

    private func saveDepthMap(_ pixelBuffer: CVPixelBuffer, to url: URL) {
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)

        guard let baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer) else { return }
        let data = Data(bytes: baseAddress, count: height * bytesPerRow)
        try? data.write(to: url)
    }

    private func saveConfidenceMap(_ pixelBuffer: CVPixelBuffer, to url: URL) {
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        let height = CVPixelBufferGetHeight(pixelBuffer)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)

        guard let baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer) else { return }
        let data = Data(bytes: baseAddress, count: height * bytesPerRow)
        try? data.write(to: url)
    }

    private func savePixelBufferAsJPEG(_ pixelBuffer: CVPixelBuffer, to url: URL) async {
        // TODO: Convert CVPixelBuffer (YCbCr) → JPEG via CIImage + CIContext
    }

    private func matrixToArray(_ m: simd_float4x4) -> [[Float]] {
        [
            [m.columns.0.x, m.columns.1.x, m.columns.2.x, m.columns.3.x],
            [m.columns.0.y, m.columns.1.y, m.columns.2.y, m.columns.3.y],
            [m.columns.0.z, m.columns.1.z, m.columns.2.z, m.columns.3.z],
            [m.columns.0.w, m.columns.1.w, m.columns.2.w, m.columns.3.w]
        ]
    }

    private func intrinsicsToArray(_ m: simd_float3x3) -> [[Float]] {
        [
            [m.columns.0.x, m.columns.1.x, m.columns.2.x],
            [m.columns.0.y, m.columns.1.y, m.columns.2.y],
            [m.columns.0.z, m.columns.1.z, m.columns.2.z]
        ]
    }
}

struct ScanPose: Codable {
    let timestamp: Double
    let transformMatrix: [[Float]]
    let intrinsics: [[Float]]
    let imageWidth: Int
    let imageHeight: Int
    let depthWidth: Int
    let depthHeight: Int
    let insta360Captures: [Insta360CaptureRef]

    enum CodingKeys: String, CodingKey {
        case timestamp
        case transformMatrix = "transform_matrix"
        case intrinsics
        case imageWidth = "image_width"
        case imageHeight = "image_height"
        case depthWidth = "depth_width"
        case depthHeight = "depth_height"
        case insta360Captures = "insta360_captures"
    }
}

struct Insta360CaptureRef: Codable {
    let cameraId: String
    let timestamp: Double
    let clockOffsetMs: Double

    enum CodingKeys: String, CodingKey {
        case cameraId = "camera_id"
        case timestamp
        case clockOffsetMs = "clock_offset_ms"
    }
}
