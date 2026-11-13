import Foundation
import Combine
import INSCameraSDK
import INSCoreMedia

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

        onLogMessage?(Self.sdkVersionSummary())
        let needsSetup = await MainActor.run {
            () -> Bool in
            Insta360ConnectionMonitor.shared.onLogMessage = {
                [weak self] msg in
                self?.onLogMessage?(msg)
            }
            Insta360LivePreview.shared.onLogMessage = {
                [weak self] msg in
                self?.onLogMessage?(msg)
            }
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
                    + " — is the iPhone on the camera's Wi-Fi?"
                )
                return
            }
        }

        // Reload config so newly added cameras are picked up.
        let freshConfig = MultiCameraConfig
            .loadFromDeviceOrBundle()
        let enabledCameras = freshConfig.cameras
            .filter(\.enabled)
        // The SDK connects to one camera (INSCameraManager.socket()),
        // so use one camera entry for it. An instance per enabled entry
        // sent every command, takePicture included, once per entry to
        // the same camera.
        guard let entry = await entryForConnectedCamera(
            enabledCameras)
        else {
            onLogMessage?("no enabled camera in Camera Settings")
            return
        }
        let instances = [CameraInstance(config: entry)]
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

        // The camera may still be booting: wait until a getOptions read
        // works, as the sample's page reads options before anything else.
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

        // The original app's camera settings: single photo mode on both
        // sensors, FlowState and horizon leveling off. Sent before the
        // live preview starts, so the preview uses the sensors they set.
        if commandsReady {
            await withTaskGroup(of: Void.self) { group in
                for cam in connected {
                    group.addTask {
                        await cam.applyCameraSettings()
                    }
                }
            }
            onLogMessage?(
                "camera configured"
            )
        }

        // Then start the live preview, as the sample's take-picture page
        // does when it opens. No warmup photo: the sample takes none.
        // Shots wait for this task.
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
                    await cam.applyCameraSettings()
                }
                self?.onLogMessage?(
                    "camera configured (deferred)"
                )
            }
            guard !Task.isCancelled else { return }
            await self?.startLivePreview()
        }
    }

    /// Starts the live preview, then lets shots through.
    private func startLivePreview() async {
        // Shows where photos are saved (X6: internal or SD card).
        let cameras = await MainActor.run { connectedCameras }
        for cam in cameras {
            await cam.logStorageStatus()
        }
        onLogMessage?("starting live preview…")
        let running = await Insta360LivePreview.shared.start()
        onLogMessage?(
            running
                ? "ready"
                : "ready (live preview did not start)"
        )
        await MainActor.run { isPipelineReady = true }
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

        // One SDK connection carries one camera, so retry only when no
        // camera is connected, with the entry that matches the camera.
        let anyConnected = await MainActor.run {
            !connectedCameras.isEmpty
        }
        guard !anyConnected,
              let entry = await entryForConnectedCamera(
                  cameraConfig.cameras.filter(\.enabled))
        else { return }

        let instance = CameraInstance(config: entry)
        instance.onStatusUpdate = { [weak self] msg in
            self?.onLogMessage?(msg)
        }
        guard await instance.reconnect() else { return }
        let camId = instance.id
        instance.onDisconnect = { [weak self] in
            self?.handleDisconnect(cameraId: camId)
        }

        await MainActor.run { connectedCameras.append(instance) }
        await startLivePreview()
    }

    /// Camera entry for the camera the SDK is connected to: the enabled
    /// entry whose serial number matches, else the first enabled entry.
    private func entryForConnectedCamera(
        _ entries: [CameraConfig]
    ) async -> CameraConfig? {
        guard let first = entries.first else { return nil }
        var serial = ""
        var name = ""
        // Device info can arrive just after the socket connects. Wait
        // for it (up to 3 s) only when there is an entry to choose.
        for _ in 0..<12 {
            let info = await MainActor.run { () -> (String, String) in
                let cam = INSCameraManager.socket().currentCamera
                return (cam?.serialNumber ?? "", cam?.name ?? "")
            }
            serial = info.0
            name = info.1
            if !serial.isEmpty || entries.count == 1 { break }
            try? await Task.sleep(for: .milliseconds(250))
        }
        let key = Self.normalizedSerial(serial)
        let match: CameraConfig? = key.isEmpty
            ? nil
            : entries.first(where: {
                Self.normalizedSerial($0.serial) == key
            })
        let entry = match ?? first
        var msg = "camera \(name.isEmpty ? "?" : name)"
            + " serial \(serial.isEmpty ? "?" : serial)"
            + " -> \(entry.id) (\(entry.model))"
        if match == nil, entries.count > 1 {
            msg += ", no entry has this serial: using the first"
        }
        onLogMessage?(msg)
        return entry
    }

    private static func normalizedSerial(_ s: String) -> String {
        s.trimmingCharacters(in: .whitespacesAndNewlines)
            .uppercased()
    }

    /// Versions of the linked Insta360 frameworks, read the way the SDK
    /// sample's root screen reads them. SDK package 1.11.3 ships
    /// INSCameraSDK 1.9.0 (482), INSCameraServiceSDK 0.2500.19 (108)
    /// and INSCoreMedia 2.878.4 (2384). None of them reports "1.11.3".
    static func sdkVersionSummary() -> String {
        func version(_ bundle: Bundle) -> String {
            let info = bundle.infoDictionary
            let short = info?["CFBundleShortVersionString"]
                as? String ?? "?"
            let build = info?["CFBundleVersion"] as? String ?? "?"
            return "\(short) (\(build))"
        }
        let service = version(Bundle(for: INSCameraManager.self))
        var summary = "Insta360 SDK: INSCameraSDK "
            + version(Bundle(for: INSCameraMediaSession.self))
            + ", INSCameraServiceSDK \(service)"
            + ", INSCoreMedia "
            + version(Bundle(for: INSExtraMetadata.self))
        if service != "0.2500.19 (108)" {
            summary += " (not the SDK 1.11.3 frameworks)"
        }
        return summary
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
        await Insta360LivePreview.shared.stop()
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

    func awaitWarmup() async {
        if let task = warmupTask {
            onLogMessage?("waiting for the live preview to start…")
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
        skipPreChecks: Bool = false
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
                        skipPreChecks: skipPreChecks
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
