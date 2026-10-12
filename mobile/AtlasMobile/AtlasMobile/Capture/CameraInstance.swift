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
        return await withUnsafeContinuation { continuation in
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
                    guard let state = change.newValue else { return }
                    DispatchQueue.main.async { [weak self] in
                        guard let self else { return }
                        self.onStatusUpdate?(
                            "[\(self.id)] State → \(Self.stateName(state))"
                        )
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
        onDisconnect = nil
        onStatusUpdate = nil
        isConnected = false
        pendingURIs.removeAll()
    }

    func clearPending() {
        pendingURIs.removeAll()
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
        var didResume = false
        return await withUnsafeContinuation { (continuation: UnsafeContinuation<ClockOffset, Never>) in
            func finish(_ value: ClockOffset) {
                guard !didResume else { return }
                didResume = true
                continuation.resume(returning: value)
            }

            let fallback = ClockOffset(
                offsetMs: 0, sampleCount: 0, stdDevMs: 0
            )

            INSCameraManager.shared().commandManager.syncTimeMsToCamera(
                withTryCount: kClockSyncSamples,
                dTimeMsMax: 200
            ) { [weak self] dTimeMs, error in
                let offset = error == nil
                    ? ClockOffset(
                        offsetMs: Double(dTimeMs),
                        sampleCount: Int(kClockSyncSamples),
                        stdDevMs: 0.0)
                    : fallback
                self?.clockOffset = offset
                DispatchQueue.main.async { finish(offset) }
            }

            DispatchQueue.main.asyncAfter(deadline: .now() + 15) {
                print("[Insta360] clock calibration timeout")
                finish(fallback)
            }
        }
    }

    // MARK: - Capture

    func capture(arkitTimestamp: Double, scanIndex: Int, timeout: Double = 15) async -> Insta360CaptureResult? {
        let insta360Ts = Date().timeIntervalSince1970
        let options = INSTakePictureOptions()

        var didResume = false
        let uri: String? = await withUnsafeContinuation { (continuation: UnsafeContinuation<String?, Never>) in
            func finish(_ value: String?) {
                guard !didResume else { return }
                didResume = true
                continuation.resume(returning: value)
            }

            DispatchQueue.main.async {
                INSCameraManager.shared().commandManager
                    .takePicture(with: options) { error, photoInfo in
                        if let error {
                            print("[Insta360] capture error: \(error)")
                        }
                        DispatchQueue.main.async { finish(photoInfo?.uri) }
                    }
            }

            DispatchQueue.main.asyncAfter(deadline: .now() + timeout) {
                print("[Insta360] capture timeout (\(timeout)s)")
                finish(nil)
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

        var results: [Insta360DownloadResult] = []
        for item in toDownload {
            if let r = await downloadOne(
                scanIndex: item.scanIndex,
                uri: item.uri,
                into: directory
            ) {
                results.append(r)
            }
        }
        return results
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
        try? FileManager.default.removeItem(at: destURL)

        let cameraId = self.id
        var didResume = false
        return await withUnsafeContinuation { (continuation: UnsafeContinuation<Insta360DownloadResult?, Never>) in
            func finish(_ value: Insta360DownloadResult?) {
                guard !didResume else { return }
                didResume = true
                continuation.resume(returning: value)
            }

            INSCameraHTTPManager.socket().fetchResource(
                withURI: uri,
                toLocalFile: destURL,
                progress: { _ in }
            ) { error in
                DispatchQueue.main.async {
                    if let error {
                        print("[Insta360] download failed: \(error)")
                        finish(nil)
                        return
                    }
                    finish(Insta360DownloadResult(
                        cameraId: cameraId,
                        scanIndex: scanIndex,
                        localURL: destURL,
                        mediaType: .equirectangular
                    ))
                }
            }

            DispatchQueue.main.asyncAfter(deadline: .now() + 30) {
                print("[Insta360] download timeout (30s)")
                finish(nil)
            }
        }
    }

    private func startHeartbeat() {
        heartbeatTimer = Timer.scheduledTimer(
            withTimeInterval: kHeartbeatInterval,
            repeats: true
        ) { [weak self] _ in
            guard self?.isConnected == true else { return }
            INSCameraManager.shared().commandManager
                .sendHeartbeats(with: nil)
        }
    }

    private func stopHeartbeat() {
        heartbeatTimer?.invalidate()
        heartbeatTimer = nil
    }

    private static let disconnectStates: Set<INSCameraState> = [
        .connectFailed, .noConnection,
    ]

    private func monitorConnection() {
        disconnectToken = INSCameraManager.socket().observe(
            \.cameraState, options: [.new]
        ) { [weak self] _, change in
            guard let state = change.newValue else { return }
            DispatchQueue.main.async { [weak self] in
                guard let self, self.isConnected,
                      Self.disconnectStates.contains(state)
                else { return }
                print("[Insta360] disconnect detected: \(Self.stateName(state))")
                self.isConnected = false
                self.stopHeartbeat()
                self.disconnectToken = nil
                self.onDisconnect?()
            }
        }
    }
}
