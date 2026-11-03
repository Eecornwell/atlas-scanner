import Foundation
import Combine
import INSCameraSDK

/// Manages connections and capture for multiple Insta360 cameras.
final class Insta360CaptureManager: ObservableObject {
    @Published var connectedCameras: [CameraInstance] = []
    @Published var isConnecting = false
    @Published var isPipelineReady = false

    var onCameraDisconnected: ((String) -> Void)?
    var onLogMessage: ((String) -> Void)?

    private var warmupTask: Task<Void, Never>?

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
            return String(
                format: "%@%dms ±%.0fms",
                sign, Int(o.offsetMs), o.uncertaintyMs
            )
        }
        return "sync: " + descriptions.joined(separator: ", ")
    }

    // MARK: - Session lifecycle

    func discoverAndConnect() async {
        await MainActor.run {
            isConnecting = true
            isPipelineReady = false
        }
        defer { Task { @MainActor in self.isConnecting = false } }

        let needsSetup = await MainActor.run {
            () -> Bool in
            let socket = INSCameraManager.socket()
            for cam in connectedCameras {
                cam.onDisconnect = nil
            }
            connectedCameras = []
            if socket.cameraState == .connected {
                return false
            }
            socket.setup()
            return true
        }
        if needsSetup {
            let ok = await waitForSDKConnected(
                maxWait: 20
            )
            if !ok {
                onLogMessage?(
                    "SDK not connected after 20s"
                    + " — is USB plugged in?"
                )
                return
            }
        }

        // Reload config so newly added cameras are picked up.
        let freshConfig = MultiCameraConfig
            .loadFromDeviceOrBundle()
        let enabledCameras = freshConfig.cameras
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

        for cam in connected {
            cam.onDisconnect = { [weak self] in
                self?.handleDisconnect(cameraId: cam.id)
            }
        }
        await MainActor.run { connectedCameras = connected }

        var commandsReady = false
        if let first = connected.first {
            commandsReady = await first.waitForCommandReady(
                maxWait: 90
            )
            onLogMessage?(
                "command channel: "
                + (commandsReady ? "ready" : "timeout")
            )
        }

        if commandsReady {
            await withTaskGroup(of: Void.self) { group in
                for cam in connected {
                    group.addTask {
                        await cam.switchToPhotoMode()
                        await cam.disableStabilization()
                    }
                }
            }
            onLogMessage?(
                "camera configured"
            )
        }

        warmupTask = Task { [weak self] in
            guard let first = connected.first
            else { return }

            if !commandsReady {
                self?.onLogMessage?(
                    "waiting for camera boot…"
                )
                let ready = await first
                    .waitForCommandReady(maxWait: 120)
                guard ready else {
                    self?.onLogMessage?(
                        "camera unresponsive"
                    )
                    return
                }
                for cam in connected {
                    await cam.switchToPhotoMode()
                    await cam.disableStabilization()
                }
                self?.onLogMessage?(
                    "camera configured (deferred)"
                )
            }

            self?.stopInFlightCapture()
            var primed = await first.warmupCapture()
            if !primed {
                self?.stopInFlightCapture()
                self?.onLogMessage?(
                    "warmup busy — retry in 3s…"
                )
                try? await Task.sleep(
                    for: .seconds(3)
                )
                primed = await first.warmupCapture()
            }
            await MainActor.run {
                self?.isPipelineReady = true
            }
            self?.onLogMessage?(
                primed
                    ? "pipeline primed — ready"
                    : "pipeline prime failed"
                    + " — proceeding"
            )

            self?.onLogMessage?(
                "ready"
            )
        }

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

    private func waitForSDKConnected(
        maxWait: Double
    ) async -> Bool {
        let start = Date()
        while Date().timeIntervalSince(start) < maxWait
        {
            let state = await MainActor.run {
                INSCameraManager.socket().cameraState
            }
            if state == .connected { return true }
            let left = Int(
                maxWait
                    - Date().timeIntervalSince(start)
            )
            onLogMessage?(
                "SDK waiting… \(left)s left"
            )
            try? await Task.sleep(for: .seconds(1))
        }
        return false
    }

    private func handleDisconnect(cameraId: String) {
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.connectedCameras.removeAll { $0.id == cameraId }
            self.onCameraDisconnected?(cameraId)
        }
    }

    func disconnect() async {
        warmupTask?.cancel()
        warmupTask = nil
        stopInFlightCapture()
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

    func stopInFlightCapture() {
        DispatchQueue.main.async {
            let socket = INSCameraManager.socket()
            guard socket.cameraState == .connected else {
                return
            }
            INSCameraManager.shared()
                .commandManager
                .stopTakePicture { _ in }
        }
    }

    func awaitWarmup() async {
        if let task = warmupTask {
            onLogMessage?("awaiting pipeline warmup…")
            await task.value
            warmupTask = nil
        }
    }

    func prepareCameras() async {
        await awaitWarmup()
        let cameras = await MainActor.run {
            connectedCameras
        }
        await withTaskGroup(of: Void.self) { group in
            for cam in cameras {
                group.addTask {
                    _ = await cam.prepareForCapture()
                }
            }
        }
    }

    func captureAll(arkitTimestamp: Double, scanIndex: Int, timeout: Double = 8, skipPreChecks: Bool = false) async -> [Insta360CaptureResult] {
        let cameras = await MainActor.run { connectedCameras }
        return await captureWith(
            cameras: cameras, arkitTimestamp: arkitTimestamp,
            scanIndex: scanIndex, timeout: timeout,
            skipPreChecks: skipPreChecks
        )
    }

    func captureWith(
        cameras: [CameraInstance],
        arkitTimestamp: Double, scanIndex: Int, timeout: Double = 8,
        skipPreChecks: Bool = false,
        retryOnFailure: Bool = true
    ) async -> [Insta360CaptureResult] {
        guard !cameras.isEmpty, !Task.isCancelled else { return [] }

        if !skipPreChecks {
            await awaitWarmup()
        }
        return await withTaskGroup(of: Insta360CaptureResult?.self) { group in
            for camera in cameras {
                guard !Task.isCancelled else { break }
                group.addTask {
                    await camera.capture(
                        arkitTimestamp: arkitTimestamp,
                        scanIndex: scanIndex,
                        timeout: timeout,
                        skipPreChecks: skipPreChecks,
                        retryOnFailure: retryOnFailure
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

    /// Shots still queued for download across connected cameras.
    func pendingDownloadCount() async -> Int {
        let cameras = await MainActor.run { connectedCameras }
        return cameras.reduce(0) { $0 + $1.pendingDownloadCount }
    }

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

    func saveClockOffsets(
        to sessionDirectory: URL,
        phase: String = "start"
    ) {
        let entries = connectedCameras.compactMap {
            cam -> ClockOffsetEntry? in
            guard let o = cam.clockOffset else { return nil }
            let preSetRaw: RawSyncSample? = o.preSetSample
                .map {
                    RawSyncSample(
                        phoneSendMs: $0.phoneSendMs,
                        phoneRecvMs: $0.phoneRecvMs,
                        cameraMs: $0.cameraMs,
                        offsetMs: $0.offsetMs,
                        roundTripMs: $0.roundTripMs
                    )
                }
            return ClockOffsetEntry(
                cameraId: cam.id,
                phase: phase,
                offsetMs: o.offsetMs,
                uncertaintyMs: o.uncertaintyMs,
                sampleCount: o.sampleCount,
                wallClockSec: o.wallClockSec,
                arkitTimeSec: o.arkitTimeSec,
                preSetSample: preSetRaw,
                samples: o.samples.map {
                    RawSyncSample(
                        phoneSendMs: $0.phoneSendMs,
                        phoneRecvMs: $0.phoneRecvMs,
                        cameraMs: $0.cameraMs,
                        offsetMs: $0.offsetMs,
                        roundTripMs: $0.roundTripMs
                    )
                }
            )
        }
        let url = sessionDirectory.appendingPathComponent(
            "calibration/clock_offset_samples.json"
        )
        var existing: [ClockOffsetEntry] = []
        if let data = try? Data(contentsOf: url),
           let prev = try? JSONDecoder().decode(
               [ClockOffsetEntry].self, from: data
           )
        {
            existing = prev
        }
        existing.append(contentsOf: entries)
        let encoder = JSONEncoder()
        encoder.outputFormatting = [
            .prettyPrinted, .sortedKeys,
        ]
        if let data = try? encoder.encode(existing) {
            try? data.write(to: url)
        }
    }
}

private struct ClockOffsetEntry: Codable {
    let cameraId: String
    let phase: String
    let offsetMs: Double
    let uncertaintyMs: Double
    let sampleCount: Int
    let wallClockSec: Double
    let arkitTimeSec: Double
    let preSetSample: RawSyncSample?
    let samples: [RawSyncSample]

    enum CodingKeys: String, CodingKey {
        case cameraId = "camera_id"
        case phase
        case offsetMs = "offset_ms"
        case uncertaintyMs = "uncertainty_ms"
        case sampleCount = "sample_count"
        case wallClockSec = "wall_clock_sec"
        case arkitTimeSec = "arkit_time_sec"
        case preSetSample = "pre_set_sample"
        case samples
    }
}

private struct RawSyncSample: Codable {
    let phoneSendMs: Double
    let phoneRecvMs: Double
    let cameraMs: Double
    let offsetMs: Double
    let roundTripMs: Double

    enum CodingKeys: String, CodingKey {
        case phoneSendMs = "phone_send_ms"
        case phoneRecvMs = "phone_recv_ms"
        case cameraMs = "camera_ms"
        case offsetMs = "offset_ms"
        case roundTripMs = "round_trip_ms"
    }
}
