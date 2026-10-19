import Foundation
import Combine
import INSCameraSDK

/// Manages connections and capture for multiple Insta360 cameras.
final class Insta360CaptureManager: ObservableObject {
    @Published var connectedCameras: [CameraInstance] = []
    @Published var isConnecting = false

    var onCameraDisconnected: ((String) -> Void)?
    var onLogMessage: ((String) -> Void)?

    let cameraConfig: MultiCameraConfig

    init(config: MultiCameraConfig) {
        self.cameraConfig = config
    }

    convenience init() {
        self.init(config: MultiCameraConfig.loadFromDeviceOrBundle())
    }

    // MARK: - Thread-safe accessors

    @MainActor
    func connectedCount() -> Int {
        connectedCameras.count
    }

    @MainActor
    func connectedCamerasCopy() -> [CameraInstance] {
        connectedCameras
    }

    @MainActor
    func clockOffsetSummary() -> String? {
        let offsets = connectedCameras.compactMap { $0.clockOffset }
        guard !offsets.isEmpty else { return nil }
        let descriptions = offsets.map { o in
            if o.sampleCount == 0 {
                return "timeout"
            }
            let sign = o.offsetMs >= 0 ? "+" : ""
            return "\(sign)\(Int(o.offsetMs))ms"
        }
        return "sync: " + descriptions.joined(separator: ", ")
    }

    // MARK: - Session lifecycle

    func discoverAndConnect() async {
        await MainActor.run { isConnecting = true }
        defer { Task { @MainActor in self.isConnecting = false } }

        // Re-trigger USB discovery so the SDK picks up cameras
        // added after the initial onAppear setup() call.
        await MainActor.run {
            let socket = INSCameraManager.socket()
            if socket.cameraState != .connected {
                socket.setup()
            }
        }
        // Give the SDK time to discover the camera on USB.
        try? await Task.sleep(for: .seconds(2))

        let enabledCameras = cameraConfig.cameras
            .filter(\.enabled)
        let instances = enabledCameras
            .map { CameraInstance(config: $0) }
        for instance in instances {
            instance.onStatusUpdate = { [weak self] msg in
                self?.onLogMessage?(msg)
            }
        }

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

        await withTaskGroup(of: Void.self) { group in
            for cam in connected {
                group.addTask {
                    _ = await cam.calibrateClockOffset()
                    await cam.disableStabilization()
                }
            }
        }

        for cam in connected {
            cam.onDisconnect = { [weak self] in
                self?.handleDisconnect(cameraId: cam.id)
            }
        }

        await MainActor.run { connectedCameras = connected }
    }

    func retryConnection() async {
        await MainActor.run { isConnecting = true }
        defer { Task { @MainActor in self.isConnecting = false } }

        await MainActor.run {
            let socket = INSCameraManager.socket()
            if socket.cameraState != .connected {
                socket.setup()
            }
        }
        try? await Task.sleep(for: .seconds(2))

        let connectedIds = Set(await MainActor.run { connectedCameras.map(\.id) })
        let missing = cameraConfig.cameras.filter { $0.enabled && !connectedIds.contains($0.id) }
        guard !missing.isEmpty else { return }

        let reconnected: [CameraInstance] = await withTaskGroup(of: CameraInstance?.self) { group in
            for config in missing {
                group.addTask { [weak self] in
                    let instance = CameraInstance(config: config)
                    instance.onStatusUpdate = { msg in
                        self?.onLogMessage?(msg)
                    }
                    return await instance.reconnect() ? instance : nil
                }
            }
            var result: [CameraInstance] = []
            for await cam in group {
                if let cam { result.append(cam) }
            }
            return result
        }

        for cam in reconnected {
            cam.onDisconnect = { [weak self] in
                self?.handleDisconnect(cameraId: cam.id)
            }
        }

        await MainActor.run { connectedCameras.append(contentsOf: reconnected) }
    }

    private func handleDisconnect(cameraId: String) {
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.connectedCameras.removeAll { $0.id == cameraId }
            self.onCameraDisconnected?(cameraId)
        }
    }

    func disconnect() async {
        let cameras = await MainActor.run {
            let current = connectedCameras
            connectedCameras = []
            return current
        }
        await withTaskGroup(of: Void.self) { group in
            for camera in cameras {
                group.addTask { await camera.disconnect() }
            }
        }
    }

    // MARK: - Capture

    func clearCaptureState() async {
        let cameras = await MainActor.run { connectedCameras }
        for camera in cameras {
            camera.clearCaptureState()
        }
    }

    func captureAll(arkitTimestamp: Double, scanIndex: Int, timeout: Double = 8) async -> [Insta360CaptureResult] {
        let cameras = await MainActor.run { connectedCameras }
        return await captureWith(
            cameras: cameras, arkitTimestamp: arkitTimestamp,
            scanIndex: scanIndex, timeout: timeout
        )
    }

    func captureWith(
        cameras: [CameraInstance],
        arkitTimestamp: Double, scanIndex: Int, timeout: Double = 8
    ) async -> [Insta360CaptureResult] {
        guard !cameras.isEmpty, !Task.isCancelled else { return [] }
        return await withTaskGroup(of: Insta360CaptureResult?.self) { group in
            for camera in cameras {
                guard !Task.isCancelled else { break }
                group.addTask {
                    await camera.capture(
                        arkitTimestamp: arkitTimestamp,
                        scanIndex: scanIndex,
                        timeout: timeout
                    )
                }
            }
            var results: [Insta360CaptureResult] = []
            for await result in group {
                if let r = result { results.append(r) }
            }
            return results
        }
    }

    // MARK: - Download

    func downloadAllPending(into sessionDirectory: URL) async -> [Insta360DownloadResult] {
        let cameras = await MainActor.run { connectedCameras }
        guard !cameras.isEmpty else { return [] }
        let rawDir = sessionDirectory.appendingPathComponent("raw")
        return await withTaskGroup(of: [Insta360DownloadResult].self) { group in
            for camera in cameras {
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
