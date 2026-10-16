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

    var onDisconnect: (() -> Void)?
    var onStatusUpdate: ((String) -> Void)?

    // MARK: - Thread-safe state

    private let lock = NSLock()
    private var _isConnected = false
    private var _pendingURIs: [(scanIndex: Int, uri: String)] = []
    private var _connectCont: CheckedContinuation<Bool, Never>?
    private var _clockCont: CheckedContinuation<ClockOffset, Never>?
    private var _captureCont: CheckedContinuation<String?, Never>?
    private var _captureInFlight = false
    private var _captureGen: UInt64 = 0
    private var _downloadCont: CheckedContinuation<
        Insta360DownloadResult?, Never
    >?

    private(set) var isConnected: Bool {
        get {
            lock.lock()
            defer { lock.unlock() }
            return _isConnected
        }
        set {
            lock.lock()
            defer { lock.unlock() }
            _isConnected = newValue
        }
    }

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

    deinit {
        stopHeartbeat()
    }

    // MARK: - Connection

    func connect() async -> Bool {
        onStatusUpdate?(
            "[\(id)] Waiting for SDK connection..."
        )

        return await withCheckedContinuation { continuation in
            lock.lock()
            _connectCont = continuation
            lock.unlock()

            func finish(_ result: Bool) {
                self.lock.lock()
                let cont = self._connectCont
                self._connectCont = nil
                self.lock.unlock()
                cont?.resume(returning: result)
            }

            DispatchQueue.main.async { [weak self] in
                guard let self else {
                    finish(false)
                    return
                }

                let socket = INSCameraManager.socket()
                let currentState = socket.cameraState
                self.onStatusUpdate?(
                    "[\(self.id)] state="
                        + "\(Self.stateName(currentState))"
                )

                if currentState == .connected {
                    self.onStatusUpdate?(
                        "[\(self.id)] Already connected!"
                    )
                    self.isConnected = true
                    self.startHeartbeat()
                    self.monitorConnection()
                    finish(true)
                    return
                }

                if currentState == .connectFailed {
                    self.onStatusUpdate?(
                        "[\(self.id)] SDK in ConnectFailed state"
                    )
                    finish(false)
                    return
                }

                self.kvoToken = socket.observe(
                    \.cameraState,
                    options: [.new]
                ) { [weak self] _, change in
                    guard let state = change.newValue else {
                        return
                    }
                    DispatchQueue.main.async { [weak self] in
                        guard let self else { return }
                        self.onStatusUpdate?(
                            "[\(self.id)] State -> "
                                + Self.stateName(state)
                        )
                        if state == .connected {
                            self.kvoToken = nil
                            self.isConnected = true
                            self.startHeartbeat()
                            self.monitorConnection()
                            self.onStatusUpdate?(
                                "[\(self.id)] Connected!"
                            )
                            finish(true)
                        } else if state == .connectFailed {
                            self.kvoToken = nil
                            finish(false)
                        }
                    }
                }

                DispatchQueue.main.asyncAfter(
                    deadline: .now() + 30
                ) { [weak self] in
                    guard let self else { return }
                    self.kvoToken = nil
                    self.onStatusUpdate?(
                        "[\(self.id)] Timeout waiting"
                            + " for connection"
                    )
                    finish(false)
                }
            }
        }
    }

    private static func stateName(
        _ state: INSCameraState
    ) -> String {
        switch state {
        case .found: return "Found"
        case .synchronized: return "Synchronized"
        case .connected: return "Connected"
        case .connectFailed: return "ConnectFailed"
        case .noConnection: return "NoConnection"
        @unknown default:
            return "unknown(\(state.rawValue))"
        }
    }

    func disconnect() async {
        stopHeartbeat()
        kvoToken = nil
        disconnectToken = nil
        onDisconnect = nil
        onStatusUpdate = nil
        resumeAllContinuations(clearURIs: true)
    }

    func clearPending() {
        lock.lock()
        _pendingURIs.removeAll()
        lock.unlock()
    }

    func clearCaptureState() {
        lock.lock()
        _captureGen += 1
        let cont = _captureCont
        _captureCont = nil
        _captureInFlight = false
        lock.unlock()
        cont?.resume(returning: nil)
    }

    func reconnect(maxRetries: Int = 3) async -> Bool {
        await disconnect()
        for attempt in 1...maxRetries {
            if await connect() {
                _ = await calibrateClockOffset()
                return true
            }
            if attempt < maxRetries {
                try? await Task.sleep(
                    nanoseconds: 2_000_000_000
                )
            }
        }
        return false
    }

    // MARK: - Clock offset

    func calibrateClockOffset() async -> ClockOffset {
        let fallback = ClockOffset(
            offsetMs: 0, sampleCount: 0, stdDevMs: 0
        )

        return await withCheckedContinuation {
            continuation in
            lock.lock()
            _clockCont = continuation
            lock.unlock()

            func finish(_ value: ClockOffset) {
                self.lock.lock()
                let cont = self._clockCont
                self._clockCont = nil
                self.lock.unlock()
                cont?.resume(returning: value)
            }

            DispatchQueue.main.async {
                INSCameraManager.shared()
                    .commandManager
                    .syncTimeMsToCamera(
                        withTryCount: kClockSyncSamples,
                        dTimeMsMax: 200
                    ) { [weak self] dTimeMs, error in
                        let offset = error == nil
                            ? ClockOffset(
                                offsetMs: Double(dTimeMs),
                                sampleCount: Int(
                                    kClockSyncSamples
                                ),
                                stdDevMs: 0.0)
                            : fallback
                        self?.clockOffset = offset
                        DispatchQueue.main.async {
                            finish(offset)
                        }
                    }
            }

            DispatchQueue.main.asyncAfter(
                deadline: .now() + 15
            ) {
                print(
                    "[Insta360] clock calibration timeout"
                )
                finish(fallback)
            }
        }
    }

    // MARK: - Capture

    func capture(
        arkitTimestamp: Double,
        scanIndex: Int,
        timeout: Double = 15
    ) async -> Insta360CaptureResult? {
        guard !Task.isCancelled else { return nil }

        lock.lock()
        let alreadyBusy = _captureInFlight
        if !alreadyBusy { _captureInFlight = true }
        lock.unlock()
        if alreadyBusy {
            onStatusUpdate?(
                "[\(id)] capture skipped — in-flight"
            )
            return nil
        }

        let sdkState = await MainActor.run {
            INSCameraManager.socket().cameraState
        }
        if sdkState != .connected {
            onStatusUpdate?(
                "[\(id)] capture skipped — SDK state "
                + Self.stateName(sdkState)
            )
            lock.lock()
            _captureInFlight = false
            lock.unlock()
            return nil
        }

        onStatusUpdate?(
            "[\(id)] takePicture sending…"
        )

        let insta360Ts = Date().timeIntervalSince1970
        let options = INSTakePictureOptions()

        let uri: String? = await withTaskCancellationHandler {
            await withCheckedContinuation {
                continuation in
                self.lock.lock()
                self._captureGen += 1
                let gen = self._captureGen
                self._captureCont = continuation
                self.lock.unlock()

                DispatchQueue.main.async {
                    INSCameraManager.shared()
                        .commandManager
                        .takePicture(
                            with: options
                        ) { error, photoInfo in
                            let uri = photoInfo?.uri
                            self.onStatusUpdate?(
                                "[\(self.id)] takePicture "
                                + "cb: gen=\(gen) uri="
                                + "\(uri ?? "nil")"
                                + (error.map {
                                    " err=\($0)"
                                } ?? "")
                            )
                            DispatchQueue.main.async {
                                self.lock.lock()
                                guard gen == self._captureGen
                                else {
                                    self.lock.unlock()
                                    self.onStatusUpdate?(
                                        "[\(self.id)] stale cb"
                                        + " ignored (gen"
                                        + " \(gen))"
                                    )
                                    return
                                }
                                let cont = self._captureCont
                                self._captureCont = nil
                                self._captureInFlight = false
                                self.lock.unlock()
                                cont?.resume(
                                    returning: uri
                                )
                            }
                        }
                }

                DispatchQueue.main.asyncAfter(
                    deadline: .now() + timeout
                ) {
                    self.lock.lock()
                    guard gen == self._captureGen else {
                        self.lock.unlock()
                        self.onStatusUpdate?(
                            "[\(self.id)] stale timeout"
                            + " ignored (gen \(gen))"
                        )
                        return
                    }
                    let cont = self._captureCont
                    self._captureCont = nil
                    self._captureInFlight = false
                    self.lock.unlock()
                    self.onStatusUpdate?(
                        "[\(self.id)] takePicture timeout"
                        + " (\(Int(timeout))s)"
                    )
                    cont?.resume(returning: nil)
                }
            }
        } onCancel: {
            self.lock.lock()
            let cont = self._captureCont
            self._captureCont = nil
            self._captureInFlight = false
            self.lock.unlock()
            cont?.resume(returning: nil)
        }

        if let uri {
            lock.lock()
            _pendingURIs.append(
                (scanIndex: scanIndex, uri: uri)
            )
            lock.unlock()
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

    func downloadPending(
        into directory: URL
    ) async -> [Insta360DownloadResult] {
        lock.lock()
        let connected = _isConnected
        let toDownload = _pendingURIs
        _pendingURIs.removeAll()
        lock.unlock()

        guard connected, !toDownload.isEmpty else {
            return []
        }

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

    private func resumeAllContinuations(
        clearURIs: Bool
    ) {
        let fallback = ClockOffset(
            offsetMs: 0, sampleCount: 0, stdDevMs: 0
        )

        lock.lock()
        _isConnected = false
        _captureInFlight = false
        if clearURIs { _pendingURIs.removeAll() }
        let connectCont = _connectCont
        let clockCont = _clockCont
        let captureCont = _captureCont
        let downloadCont = _downloadCont
        _connectCont = nil
        _clockCont = nil
        _captureCont = nil
        _downloadCont = nil
        lock.unlock()

        connectCont?.resume(returning: false)
        clockCont?.resume(returning: fallback)
        captureCont?.resume(returning: nil)
        downloadCont?.resume(returning: nil)
    }

    private func downloadOne(
        scanIndex: Int,
        uri: String,
        into directory: URL
    ) async -> Insta360DownloadResult? {
        let destURL = directory
            .appendingPathComponent(id)
            .appendingPathComponent(
                String(
                    format: "scan_%03d.jpg",
                    scanIndex
                )
            )

        try? FileManager.default.createDirectory(
            at: destURL.deletingLastPathComponent(),
            withIntermediateDirectories: true
        )
        try? FileManager.default.removeItem(at: destURL)

        let cameraId = self.id
        return await withCheckedContinuation {
            continuation in
            lock.lock()
            _downloadCont = continuation
            lock.unlock()

            func finish(
                _ value: Insta360DownloadResult?
            ) {
                self.lock.lock()
                let cont = self._downloadCont
                self._downloadCont = nil
                self.lock.unlock()
                cont?.resume(returning: value)
            }

            DispatchQueue.main.async {
                INSCameraHTTPManager.socket().fetchResource(
                    withURI: uri,
                    toLocalFile: destURL,
                    progress: { _ in }
                ) { error in
                    DispatchQueue.main.async {
                        if let error {
                            print(
                                "[Insta360] download "
                                    + "failed: \(error)"
                            )
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
            }

            DispatchQueue.main.asyncAfter(
                deadline: .now() + 30
            ) {
                print(
                    "[Insta360] download timeout (30s)"
                )
                finish(nil)
            }
        }
    }

    private func startHeartbeat() {
        heartbeatTimer = Timer.scheduledTimer(
            withTimeInterval: kHeartbeatInterval,
            repeats: true
        ) { [weak self] _ in
            guard self?.isConnected == true else {
                return
            }
            INSCameraManager.shared().commandManager
                .sendHeartbeats(with: nil)
        }
    }

    private func stopHeartbeat() {
        heartbeatTimer?.invalidate()
        heartbeatTimer = nil
    }

    private static let disconnectStates:
        Set<INSCameraState> = [
            .connectFailed, .noConnection,
        ]

    private func monitorConnection() {
        disconnectToken = INSCameraManager.socket()
            .observe(
                \.cameraState, options: [.new]
            ) { [weak self] _, change in
                guard let state = change.newValue else {
                    return
                }
                DispatchQueue.main.async { [weak self] in
                    guard let self, self.isConnected,
                          Self.disconnectStates
                              .contains(state)
                    else { return }
                    print(
                        "[Insta360] disconnect detected:"
                            + " \(Self.stateName(state))"
                    )
                    self.stopHeartbeat()
                    self.disconnectToken = nil
                    self.resumeAllContinuations(
                        clearURIs: false
                    )
                    self.onDisconnect?()
                }
            }
    }
}
