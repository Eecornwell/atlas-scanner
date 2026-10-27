import CoreMedia
import Foundation
import ImageIO
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
    private var _captureCont: CheckedContinuation<CaptureAck?, Never>?
    private var _captureInFlight = false
    private var _captureGen: UInt64 = 0
    private var _downloadCont: CheckedContinuation<
        Insta360DownloadResult?, Never
    >?
    private(set) var isPhotoModeSet = false

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
                await switchToPhotoMode()
                await disableStabilization()
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

    func prepareForCapture() async -> Bool {
        guard await ensureSDKConnected() else {
            return false
        }
        _ = await warmUpCommandChannel()
        await ensurePhotoMode()
        return true
    }

    func capture(
        arkitTimestamp: Double,
        scanIndex: Int,
        timeout: Double = 45,
        skipPreChecks: Bool = false,
        retryOnFailure: Bool = true
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
        defer {
            lock.lock()
            _captureInFlight = false
            lock.unlock()
        }

        if !skipPreChecks {
            guard await ensureSDKConnected() else {
                return nil
            }
        }

        if !skipPreChecks {
            _ = await warmUpCommandChannel()
            await ensurePhotoMode()
        }

        let commandTs = ProcessInfo.processInfo.systemUptime
        var captureResult: (
            uri: String, callbackTimestamp: Double
        )?
        captureResult = await attemptTakePicture(
            timeout: timeout
        )

        if captureResult == nil, retryOnFailure {
            onStatusUpdate?(
                "[\(id)] attempt 1 failed"
                + " — waiting 15s for camera"
            )
            try? await Task.sleep(
                for: .seconds(15)
            )
            let recovered = await waitForCommandReady(
                maxWait: 75
            )
            if recovered, !Task.isCancelled {
                captureResult = await attemptTakePicture(
                    timeout: timeout
                )
            }
        }

        if let uri = captureResult?.uri {
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
            commandTimestamp: commandTs,
            callbackTimestamp: captureResult?
                .callbackTimestamp ?? commandTs,
            mediaIdentifier: captureResult?.uri ?? ""
        )
    }

    private func waitForConnected(
        maxWait: Double
    ) async -> Bool {
        let start = Date()
        while Date().timeIntervalSince(start) < maxWait {
            let state = await MainActor.run {
                INSCameraManager.socket().cameraState
            }
            if state == .connected {
                onStatusUpdate?(
                    "[\(id)] reconnected after "
                    + "\(Int(Date().timeIntervalSince(start)))s"
                )
                return true
            }
            onStatusUpdate?(
                "[\(id)] waiting… state="
                + Self.stateName(state)
            )
            try? await Task.sleep(for: .seconds(1))
        }
        return false
    }

    private func ensureSDKConnected() async -> Bool {
        let state = await MainActor.run {
            INSCameraManager.socket().cameraState
        }
        if state == .connected { return true }

        onStatusUpdate?(
            "[\(id)] SDK state "
            + Self.stateName(state)
            + " — waiting"
        )
        return await waitForConnected(maxWait: 10)
    }

    private func waitForCameraReady(
        maxWait: Double = 10
    ) async -> Bool {
        let start = Date()
        while Date().timeIntervalSince(start) < maxWait {
            guard !Task.isCancelled else { return false }
            let ready = await withCheckedContinuation {
                (cont: CheckedContinuation<Bool, Never>) in
                DispatchQueue.main.async {
                    INSCameraManager.shared()
                        .commandManager
                        .getCurrentCaptureStatus {
                            error, status in
                            if let error {
                                self.onStatusUpdate?(
                                    "[\(self.id)] status"
                                    + " err: \(error)"
                                )
                                cont.resume(returning: false)
                                return
                            }
                            let state = status?.state
                                ?? .notCapture
                            if state == .notCapture {
                                cont.resume(returning: true)
                            } else {
                                self.onStatusUpdate?(
                                    "[\(self.id)] camera"
                                    + " state="
                                    + "\(state.rawValue)"
                                    + " — waiting"
                                )
                                cont.resume(returning: false)
                            }
                        }
                }
            }
            if ready { return true }
            try? await Task.sleep(for: .seconds(1))
        }
        onStatusUpdate?(
            "[\(id)] camera not ready after"
            + " \(Int(maxWait))s"
        )
        return false
    }

    func waitForCommandReady(
        maxWait: Double = 60
    ) async -> Bool {
        let start = Date()
        while Date().timeIntervalSince(start) < maxWait {
            let ok = await warmUpCommandChannel(
                timeout: 5
            )
            if ok { return true }
            let elapsed = Int(
                Date().timeIntervalSince(start)
            )
            onStatusUpdate?(
                "[\(id)] commands not ready"
                + " (\(elapsed)s) — retrying"
            )
            try? await Task.sleep(for: .seconds(1))
        }
        return false
    }

    func warmUpCommandChannel(
        timeout: Double = 15
    ) async -> Bool {
        await withCheckedContinuation {
            (cont: CheckedContinuation<Bool, Never>)
                in
            let done = NSLock()
            var didResume = false

            func finish(_ result: Bool) {
                done.lock()
                defer { done.unlock() }
                guard !didResume else { return }
                didResume = true
                cont.resume(returning: result)
            }

            DispatchQueue.main.async { [weak self] in
                let types: [NSNumber] = [
                    NSNumber(
                        value: INSCameraOptionsType
                            .storageState.rawValue
                    ),
                ]
                INSCameraManager.shared()
                    .commandManager
                    .getOptionsWithTypes(
                        types
                    ) { error, _, _ in
                        if let error {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")]"
                                + " warmup: "
                                + "\(error)"
                            )
                        } else {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")]"
                                + " warmup OK"
                            )
                        }
                        finish(error == nil)
                    }
            }

            DispatchQueue.main.asyncAfter(
                deadline: .now() + timeout
            ) { [weak self] in
                done.lock()
                let alreadyDone = didResume
                done.unlock()
                if !alreadyDone {
                    self?.onStatusUpdate?(
                        "[\(self?.id ?? "?")]"
                        + " warmup timeout"
                        + " (\(Int(timeout))s)"
                    )
                }
                finish(false)
            }
        }
    }

    private var _stopCont: CheckedContinuation<
        Void, Never
    >?

    private func stopCurrentCapture() async {
        await withCheckedContinuation {
            (continuation: CheckedContinuation<Void, Never>)
                in
            self.lock.lock()
            self._stopCont = continuation
            self.lock.unlock()

            func finish() {
                self.lock.lock()
                let cont = self._stopCont
                self._stopCont = nil
                self.lock.unlock()
                cont?.resume()
            }

            DispatchQueue.main.async {
                INSCameraManager.shared()
                    .commandManager
                    .stopTakePicture { [weak self] error in
                        if let error {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "stopCapture: \(error)"
                            )
                        }
                        finish()
                    }
            }

            DispatchQueue.main.asyncAfter(
                deadline: .now() + 3
            ) {
                finish()
            }
        }
    }

    func warmupCapture(
        timeout: Double = 15
    ) async -> Bool {
        _ = await warmUpCommandChannel(timeout: 8)
        await ensurePhotoMode()
        onStatusUpdate?(
            "[\(id)] priming pipeline…"
        )
        return await withCheckedContinuation {
            (cont: CheckedContinuation<Bool, Never>)
                in
            let done = NSLock()
            var didResume = false

            func finish(_ ok: Bool) {
                done.lock()
                defer { done.unlock() }
                guard !didResume else { return }
                didResume = true
                cont.resume(returning: ok)
            }

            DispatchQueue.main.async { [weak self] in
                let opts = INSTakePictureOptions()
                opts.mode = .normal
                let exp = INSCameraExposureOptions()
                exp.program = 3
                exp.shutterSpeed = CMTime(
                    value: 120, timescale: 1
                )
                opts.currenExposureOptions = exp
                INSCameraManager.shared()
                    .commandManager
                    .takePicture(
                        with: opts
                    ) { [weak self] error,
                        photoInfo in
                        if let error {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "warmupCapture: "
                                + "\(error)"
                            )
                        } else {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "warmupCapture OK"
                            )
                        }
                        finish(error == nil)
                    }
            }

            DispatchQueue.main.asyncAfter(
                deadline: .now() + timeout
            ) { [weak self] in
                done.lock()
                let alreadyDone = didResume
                done.unlock()
                if !alreadyDone {
                    self?.onStatusUpdate?(
                        "[\(self?.id ?? "?")] "
                        + "warmupCapture timeout"
                        + " (\(Int(timeout))s)"
                    )
                }
                finish(false)
            }
        }
    }

    private func checkSensorIsAll() async -> Bool {
        await withCheckedContinuation {
            (cont: CheckedContinuation<Bool, Never>) in
            let done = NSLock()
            var didResume = false

            func finish(_ result: Bool) {
                done.lock()
                defer { done.unlock() }
                guard !didResume else { return }
                didResume = true
                cont.resume(returning: result)
            }

            DispatchQueue.main.async { [weak self] in
                INSCameraManager.shared()
                    .commandManager
                    .getActiveSensor {
                        error, device, _, _, _ in
                        if let error {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "getActiveSensor: "
                                + "\(error)"
                            )
                            finish(false)
                            return
                        }
                        let isAll = device == .all
                        self?.onStatusUpdate?(
                            "[\(self?.id ?? "?")] "
                            + "sensor="
                            + "\(device.rawValue)"
                            + (isAll
                                ? " (all)"
                                : " (not all)")
                        )
                        finish(isAll)
                    }
            }

            DispatchQueue.main.asyncAfter(
                deadline: .now() + 10
            ) { [weak self] in
                done.lock()
                let alreadyDone = didResume
                done.unlock()
                if !alreadyDone {
                    self?.onStatusUpdate?(
                        "[\(self?.id ?? "?")] "
                        + "getActiveSensor timeout"
                    )
                }
                finish(false)
            }
        }
    }

    func switchToPhotoMode() async {
        let skipActivation = await checkSensorIsAll()

        await withCheckedContinuation {
            (continuation: CheckedContinuation<Void, Never>)
                in
            let resumed = NSLock()
            var didResume = false

            func finish() {
                resumed.lock()
                defer { resumed.unlock() }
                guard !didResume else { return }
                didResume = true
                continuation.resume()
            }

            DispatchQueue.main.async { [weak self] in
                let opts = INSCameraOptions()
                opts.photoSubMode = 6
                opts.focusSensor = .all
                opts.expectOutputType = .default
                let types: [NSNumber] = [
                    NSNumber(
                        value: INSCameraOptionsType
                            .photoSubMode.rawValue
                    ),
                    NSNumber(
                        value: INSCameraOptionsType
                            .focusSensor.rawValue
                    ),
                    NSNumber(
                        value: INSCameraOptionsType
                            .expectOutputType.rawValue
                    ),
                ]
                INSCameraManager.shared()
                    .commandManager
                    .setOptions(
                        opts,
                        forTypes: types
                    ) { [weak self] error, _ in
                        if let error {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "setPhotoMode err: "
                                + "\(error)"
                            )
                            finish()
                            return
                        }
                        self?.isPhotoModeSet = true
                        self?.onStatusUpdate?(
                            "[\(self?.id ?? "?")] "
                            + "photo mode set"
                        )

                        if skipActivation {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "sensor already .all"
                                + " — skip activation"
                            )
                            finish()
                            return
                        }

                        self?.onStatusUpdate?(
                            "[\(self?.id ?? "?")] "
                            + "activating sensors…"
                        )
                        INSCameraManager.shared()
                            .commandManager
                            .setActiveSensorWith(
                                .all
                            ) { [weak self] error,
                                _, _, _ in
                                if let error {
                                    self?.onStatusUpdate?(
                                        "[\(self?.id ?? "?")] "
                                        + "activeSensor err: "
                                        + "\(error)"
                                    )
                                } else {
                                    self?.onStatusUpdate?(
                                        "[\(self?.id ?? "?")] "
                                        + "sensor activated"
                                    )
                                }
                                finish()
                            }
                    }
            }

            DispatchQueue.main.asyncAfter(
                deadline: .now() + 90
            ) { [weak self] in
                resumed.lock()
                let alreadyDone = didResume
                resumed.unlock()
                if !alreadyDone {
                    self?.onStatusUpdate?(
                        "[\(self?.id ?? "?")] "
                        + "switchToPhotoMode timeout"
                    )
                }
                finish()
            }
        }
    }

    func disableStabilization() async {
        await withCheckedContinuation {
            (continuation: CheckedContinuation<Void, Never>)
                in
            let resumed = NSLock()
            var didResume = false

            func finish() {
                resumed.lock()
                defer { resumed.unlock() }
                guard !didResume else { return }
                didResume = true
                continuation.resume()
            }

            DispatchQueue.main.async { [weak self] in
                let opts = INSCameraOptions()
                opts.enableInternalFlowstate = false
                var types: [NSNumber] = [
                    NSNumber(
                        value: INSCameraOptionsType
                            .internalFlowstate.rawValue
                    ),
                ]
                opts.horizonIndicatorEnable = false
                types.append(
                    NSNumber(
                        value: INSCameraOptionsType
                            .horizontalEnableGlobal
                            .rawValue
                    )
                )
                INSCameraManager.shared()
                    .commandManager
                    .setOptions(
                        opts,
                        forTypes: types
                    ) { [weak self] error, _ in
                        if let error {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "disableStab err: "
                                + "\(error)"
                            )
                        } else {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "stabilization disabled"
                            )
                        }
                        finish()
                    }
            }

            DispatchQueue.main.asyncAfter(
                deadline: .now() + 5
            ) {
                finish()
            }
        }
    }

    func ensurePhotoMode() async {
        guard !isPhotoModeSet else { return }
        let ready = await warmUpCommandChannel()
        guard ready else { return }
        await switchToPhotoMode()
        await disableStabilization()
    }

    private typealias CaptureAck = (
        uri: String, callbackTimestamp: Double
    )

    private func attemptTakePicture(
        timeout: Double
    ) async -> CaptureAck? {
        return await withTaskCancellationHandler {
            await withCheckedContinuation {
                continuation in
                self.lock.lock()
                self._captureGen += 1
                let gen = self._captureGen
                self._captureCont = continuation
                self.lock.unlock()

                DispatchQueue.main.async {
                    let opts = INSTakePictureOptions()
                    opts.mode = .normal
                    let exp = INSCameraExposureOptions()
                    exp.program = 3
                    exp.shutterSpeed = CMTime(
                        value: 120, timescale: 1
                    )
                    opts.currenExposureOptions = exp
                    INSCameraManager.shared()
                        .commandManager
                        .takePicture(
                            with: opts
                        ) { error, photoInfo in
                            let cbTs = ProcessInfo
                                .processInfo.systemUptime
                            let uri = photoInfo?.uri
                            self.onStatusUpdate?(
                                "[\(self.id)] cb:"
                                + " gen=\(gen)"
                                + " uri="
                                + "\(uri ?? "nil")"
                                + (error.map {
                                    " err=\($0)"
                                } ?? "")
                            )
                            self.lock.lock()
                            guard gen
                                == self._captureGen
                            else {
                                self.lock.unlock()
                                return
                            }
                            let cont =
                                self._captureCont
                            self._captureCont = nil
                            self.lock.unlock()
                            if let uri,
                               !uri.isEmpty,
                               error == nil
                            {
                                cont?.resume(
                                    returning: CaptureAck(
                                        uri: uri,
                                        callbackTimestamp:
                                            cbTs
                                    )
                                )
                            } else {
                                cont?.resume(
                                    returning: nil
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
                        return
                    }
                    let cont = self._captureCont
                    self._captureCont = nil
                    self.lock.unlock()
                    self.onStatusUpdate?(
                        "[\(self.id)] takePicture"
                        + " timeout"
                        + " (\(Int(timeout))s)"
                    )
                    cont?.resume(returning: nil)
                }
            }
        } onCancel: {
            self.lock.lock()
            self._captureGen += 1
            let cont = self._captureCont
            self._captureCont = nil
            self.lock.unlock()
            cont?.resume(returning: nil)
        }
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
        let stopCont = _stopCont
        _connectCont = nil
        _clockCont = nil
        _captureCont = nil
        _downloadCont = nil
        _stopCont = nil
        lock.unlock()

        connectCont?.resume(returning: false)
        clockCont?.resume(returning: fallback)
        captureCont?.resume(returning: nil)
        downloadCont?.resume(returning: nil)
        stopCont?.resume()
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
                        let exifTs = Self
                            .extractExifTimestamp(
                                from: destURL
                            )
                        finish(Insta360DownloadResult(
                            cameraId: cameraId,
                            scanIndex: scanIndex,
                            localURL: destURL,
                            mediaType: .equirectangular,
                            exifTimestamp: exifTs
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

    // MARK: - EXIF

    private static let exifDateFormatter: DateFormatter = {
        let fmt = DateFormatter()
        fmt.dateFormat = "yyyy:MM:dd HH:mm:ss"
        fmt.locale = Locale(identifier: "en_US_POSIX")
        fmt.timeZone = .current
        return fmt
    }()

    static func extractExifTimestamp(
        from url: URL
    ) -> Double? {
        guard let source = CGImageSourceCreateWithURL(
            url as CFURL, nil
        ) else { return nil }
        guard let props = CGImageSourceCopyPropertiesAtIndex(
            source, 0, nil
        ) as? [String: Any] else { return nil }
        guard let exif = props[
            kCGImagePropertyExifDictionary as String
        ] as? [String: Any] else { return nil }
        guard let dateStr = exif[
            kCGImagePropertyExifDateTimeOriginal as String
        ] as? String else { return nil }
        guard let base = exifDateFormatter.date(
            from: dateStr
        ) else { return nil }
        var ts = base.timeIntervalSince1970
        if let sub = exif[
            kCGImagePropertyExifSubsecTimeOriginal as String
        ] as? String, let frac = Double("0." + sub) {
            ts += frac
        }
        return ts
    }
}
