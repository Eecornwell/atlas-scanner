import Foundation
import Combine

/// Manages connections and capture for multiple Insta360 cameras.
final class Insta360CaptureManager: ObservableObject {
    @Published var connectedCameras: [CameraInstance] = []
    @Published var isConnecting = false

    private let cameraConfig: MultiCameraConfig

    init(config: MultiCameraConfig) {
        self.cameraConfig = config
    }

    convenience init() {
        self.init(config: MultiCameraConfig.default)
    }

    func discoverAndConnect() async {
        await MainActor.run { isConnecting = true }
        defer { Task { @MainActor in isConnecting = false } }

        // TODO: Use Insta360 SDK to discover cameras via WiFi
        // For each camera in cameraConfig.cameras:
        //   1. Scan WiFi for matching serial/SSID
        //   2. Connect via SDK
        //   3. Create CameraInstance
        //   4. Calibrate clock offset
    }

    func disconnect() async {
        for camera in connectedCameras {
            await camera.disconnect()
        }
        await MainActor.run { connectedCameras = [] }
    }

    /// Triggers capture on all connected cameras and records timestamps.
    func captureAll(arkitTimestamp: Double) async -> [Insta360CaptureResult] {
        await withTaskGroup(of: Insta360CaptureResult?.self) { group in
            for camera in connectedCameras {
                group.addTask {
                    await camera.capture(arkitTimestamp: arkitTimestamp)
                }
            }

            var results: [Insta360CaptureResult] = []
            for await result in group {
                if let result { results.append(result) }
            }
            return results
        }
    }

    /// Downloads all pending media from all cameras.
    func downloadAllPending() async -> [Insta360DownloadResult] {
        await withTaskGroup(of: [Insta360DownloadResult].self) { group in
            for camera in connectedCameras {
                group.addTask {
                    await camera.downloadPending()
                }
            }

            var results: [Insta360DownloadResult] = []
            for await batch in group {
                results.append(contentsOf: batch)
            }
            return results
        }
    }
}
