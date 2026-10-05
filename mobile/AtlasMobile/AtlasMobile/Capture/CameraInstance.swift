import Foundation
import INSCameraSDK

private let kHeartbeatInterval: TimeInterval = 0.5
private let kClockSyncSamples: UInt = 10

final class CameraInstance: NSObject, Identifiable {
    let id: String
    let model: String
    let serial: String
    let extrinsic: RigidTransform
    let maskPath: String?

    private(set) var clockOffset: ClockOffset?
    private(set) var isConnected = false

    var onDisconnect: (() -> Void)?
    var onStatusUpdate: ((String) -> Void)?

    private var pendingURIs: [(scanIndex: Int, uri: String)] = []
    private var heartbeatTimer: Timer?
    private var kvoToken: NSKeyValueObservation?
    private var disconnectToken: NSKeyValueObservation?

    init(config: CameraConfig) {
        self.id = config.id
        self.model = config.model
        self.serial = config.serial
        self.extrinsic = config.extrinsic
        self.maskPath = config.mask
    }

    // MARK: - Connection

    func connect() async -> Bool {
        onStatusUpdate?("[\(id)] Waiting for SDK connection...")

        var didResume = false
        return await withCheckedContinuation { continuation in
            func finish(_ result: Bool) {
                guard !didResume else { return }
                didResume = true
                continuation.resume(returning: result)
            }

            DispatchQueue.main.async { [weak self] in
                guard let self else { finish(false); return }

                let socket = INSCameraManager.socket()
                let currentState = socket.cameraState
                self.onStatusUpdate?("[\(self.id)] state=\(Self.stateName(currentState))")

                if currentState == .connected {
                    self.onStatusUpdate?("[\(self.id)] Already connected!")
                    self.isConnected = true
                    self.startHeartbeat()
                    self.monitorConnection()
                    finish(true)
                    return
                }

                if currentState == .connectFailed {
                    self.onStatusUpdate?("[\(self.id)] SDK in ConnectFailed state")
                    finish(false)
                    return
                }

                self.kvoToken = socket.observe(
                    \.cameraState,
                    options: [.new]
                ) { [weak self] _, change in
                    guard let self, let state = change.newValue else { return }
                    self.onStatusUpdate?("[\(self.id)] State → \(Self.stateName(state))")
                    if state == .connected {
                        self.kvoToken = nil
                        self.isConnected = true
                        self.startHeartbeat()
                        self.monitorConnection()
                        self.onStatusUpdate?("[\(self.id)] Connected!")
                        finish(true)
                    } else if state == .connectFailed {
                        self.kvoToken = nil
                        finish(false)
                    }
                }

                DispatchQueue.main.asyncAfter(deadline: .now() + 30) { [weak self] in
                    guard let self, !didResume else { return }
                    self.kvoToken = nil
                    self.onStatusUpdate?("[\(self.id)] Timeout waiting for connection")
                    finish(false)
                }
            }
        }
    }

    private static func stateName(_ state: INSCameraState) -> String {
        switch state {
        case .found: return "Found"
        case .synchronized: return "Synchronized"
        case .connected: return "Connected"
        case .connectFailed: return "ConnectFailed"
        case .noConnection: return "NoConnection"
        @unknown default: return "unknown(\(state.rawValue))"
        }
    }

    func disconnect() async {
        stopHeartbeat()
        kvoToken = nil
        disconnectToken = nil
        isConnected = false
    }

    func reconnect(maxRetries: Int = 3) async -> Bool {
        await disconnect()
        for attempt in 1...maxRetries {
            if await connect() {
                _ = await calibrateClockOffset()
                return true
            }
            if attempt < maxRetries {
                try? await Task.sleep(nanoseconds: 2_000_000_000)
            }
        }
        return false
    }

    // MARK: - Clock offset

    func calibrateClockOffset() async -> ClockOffset {
        return await withCheckedContinuation { continuation in
            INSCameraManager.shared().commandManager.syncTimeMsToCamera(
                withTryCount: kClockSyncSamples,
                dTimeMsMax: 200
            ) { [weak self] dTimeMs, error in
                let offset = ClockOffset(
                    offsetMs: error == nil ? Double(dTimeMs) : 0.0,
                    sampleCount: error == nil ? Int(kClockSyncSamples) : 0,
                    stdDevMs: 0.0
                )
                self?.clockOffset = offset
                continuation.resume(returning: offset)
            }
        }
    }

    // MARK: - Capture

    func capture(arkitTimestamp: Double, scanIndex: Int) async -> Insta360CaptureResult? {
        guard isConnected else { return nil }

        let insta360Ts = Date().timeIntervalSince1970
        let options = INSTakePictureOptions()

        let uri: String? = await withCheckedContinuation { continuation in
            INSCameraManager.shared().commandManager.takePicture(with: options) { error, photoInfo in
                if let error {
                    print("[Insta360] capture error: \(error)")
                }
                continuation.resume(returning: photoInfo?.uri)
            }
        }

        if let uri {
            pendingURIs.append((scanIndex: scanIndex, uri: uri))
        }

        return Insta360CaptureResult(
            cameraId: id,
            scanIndex: scanIndex,
            arkitTimestamp: arkitTimestamp,
            insta360Timestamp: insta360Ts,
            mediaIdentifier: uri ?? ""
        )
    }

    // MARK: - Download

    func downloadPending(into directory: URL) async -> [Insta360DownloadResult] {
        guard isConnected, !pendingURIs.isEmpty else { return [] }
        let toDownload = pendingURIs
        pendingURIs.removeAll()

        return await withTaskGroup(of: Insta360DownloadResult?.self) { group in
            for item in toDownload {
                group.addTask {
                    await self.downloadOne(scanIndex: item.scanIndex, uri: item.uri, into: directory)
                }
            }
            var results: [Insta360DownloadResult] = []
            for await result in group {
                if let r = result { results.append(r) }
            }
            return results
        }
    }

    // MARK: - Private

    private func downloadOne(scanIndex: Int, uri: String, into directory: URL) async -> Insta360DownloadResult? {
        let destURL = directory
            .appendingPathComponent(id)
            .appendingPathComponent(String(format: "scan_%03d.jpg", scanIndex))

        try? FileManager.default.createDirectory(
            at: destURL.deletingLastPathComponent(),
            withIntermediateDirectories: true
        )
        // Remove stale file so fetchResource can write a fresh one.
        try? FileManager.default.removeItem(at: destURL)

        return await withCheckedContinuation { continuation in
            INSCameraHTTPManager.socket().fetchResource(
                withURI: uri,
                toLocalFile: destURL,
                progress: { _ in }
            ) { error in
                if let error {
                    print("[Insta360] download failed: \(error)")
                    continuation.resume(returning: nil)
                    return
                }
                continuation.resume(returning: Insta360DownloadResult(
                    cameraId: self.id,
                    scanIndex: scanIndex,
                    localURL: destURL,
                    mediaType: .equirectangular
                ))
            }
        }
    }

    private func startHeartbeat() {
        heartbeatTimer = Timer.scheduledTimer(withTimeInterval: kHeartbeatInterval, repeats: true) { _ in
            INSCameraManager.shared().commandManager.sendHeartbeats(with: nil)
        }
    }

    private func stopHeartbeat() {
        heartbeatTimer?.invalidate()
        heartbeatTimer = nil
    }

    private func monitorConnection() {
        disconnectToken = INSCameraManager.socket().observe(
            \.cameraState, options: [.new]
        ) { [weak self] _, change in
            guard let self, let state = change.newValue else { return }
            if state != .connected && self.isConnected {
                self.isConnected = false
                self.stopHeartbeat()
                self.disconnectToken = nil
                self.onDisconnect?()
            }
        }
    }
}
