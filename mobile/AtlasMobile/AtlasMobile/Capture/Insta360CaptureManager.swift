import Foundation
import Combine

/// Manages connections and capture for multiple Insta360 cameras.
final class Insta360CaptureManager: ObservableObject {
    @Published var connectedCameras: [CameraInstance] = []
    @Published var isConnecting = false

    let cameraConfig: MultiCameraConfig

    init(config: MultiCameraConfig) {
        self.cameraConfig = config
    }

    convenience init() {
        self.init(config: MultiCameraConfig.loadFromDeviceOrBundle())
    }

    // MARK: - Session lifecycle

    func discoverAndConnect() async {
        await MainActor.run { isConnecting = true }
        defer { Task { @MainActor in self.isConnecting = false } }

        // Attempt to connect all configured cameras in parallel.
        let instances = cameraConfig.cameras.map { CameraInstance(config: $0) }

        let connected: [CameraInstance] = await withTaskGroup(of: CameraInstance?.self) { group in
            for instance in instances {
                group.addTask {
                    let ok = await instance.connect()
                    return ok ? instance : nil
                }
            }
            var result: [CameraInstance] = []
            for await cam in group {
                if let cam { result.append(cam) }
            }
            return result
        }

        // Calibrate clock offset for each connected camera.
        await withTaskGroup(of: Void.self) { group in
            for cam in connected {
                group.addTask { _ = await cam.calibrateClockOffset() }
            }
        }

        await MainActor.run { connectedCameras = connected }
    }

    func disconnect() async {
        await withTaskGroup(of: Void.self) { group in
            for camera in connectedCameras {
                group.addTask { await camera.disconnect() }
            }
        }
        await MainActor.run { connectedCameras = [] }
    }

    // MARK: - Capture

    /// Triggers capture on all connected cameras simultaneously.
    func captureAll(arkitTimestamp: Double, scanIndex: Int) async -> [Insta360CaptureResult] {
        await withTaskGroup(of: Insta360CaptureResult?.self) { group in
            for camera in connectedCameras {
                group.addTask {
                    await camera.capture(arkitTimestamp: arkitTimestamp, scanIndex: scanIndex)
                }
            }
            var results: [Insta360CaptureResult] = []
            for await result in group {
                if let result { results.append(result) }
            }
            return results
        }
    }

    // MARK: - Download

    /// Downloads all pending media from all cameras into the session raw/ directory.
    func downloadAllPending(into sessionDirectory: URL) async -> [Insta360DownloadResult] {
        let rawDir = sessionDirectory.appendingPathComponent("raw")
        return await withTaskGroup(of: [Insta360DownloadResult].self) { group in
            for camera in connectedCameras {
                group.addTask { await camera.downloadPending(into: rawDir) }
            }
            var results: [Insta360DownloadResult] = []
            for await batch in group { results.append(contentsOf: batch) }
            return results
        }
    }

    // MARK: - Clock offset persistence

    func saveClockOffsets(to sessionDirectory: URL) {
        let samples = connectedCameras.compactMap { cam -> ClockOffsetSample? in
            guard let offset = cam.clockOffset else { return nil }
            return ClockOffsetSample(cameraId: cam.id, offsetMs: offset.offsetMs,
                                     sampleCount: offset.sampleCount, stdDevMs: offset.stdDevMs)
        }
        let url = sessionDirectory
            .appendingPathComponent("calibration/clock_offset_samples.json")
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        if let data = try? encoder.encode(samples) {
            try? data.write(to: url)
        }
    }
}

private struct ClockOffsetSample: Codable {
    let cameraId: String
    let offsetMs: Double
    let sampleCount: Int
    let stdDevMs: Double

    enum CodingKeys: String, CodingKey {
        case cameraId = "camera_id"
        case offsetMs = "offset_ms"
        case sampleCount = "sample_count"
        case stdDevMs = "std_dev_ms"
    }
}
