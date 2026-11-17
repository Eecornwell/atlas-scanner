import CoreMedia
import Foundation
import ImageIO
import INSCameraSDK
import INSCoreMedia

private let kClockSyncTotalSamples = 20
private let kClockSyncBestCount = 3

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
    /// Identifies the download that owns _downloadCont, so a late
    /// reply or timer from an earlier download cannot finish it.
    private var _downloadGen: UInt64 = 0
    /// Failed download attempts per URI; failures are re-queued until
    /// maxDownloadAttempts.
    private var _downloadAttempts: [String: Int] = [:]
    private static let maxDownloadAttempts = 3
    /// systemUptime of the running download's last progress callback.
    private var _downloadProgressAt: Double = 0
    /// A 120 MP X6 photo has 1.7 times the pixels of a 72 MP X5 photo, so
    /// a fixed limit could stop a download that is still receiving data.
    /// A download is given up after this long without progress...
    private static let downloadStallSeconds: Double = 30
    /// ...or after this long in total.
    private static let downloadMaxSeconds: Double = 180
    /// Gets the photo URI from an INSCameraTakePictureResponse push
    /// while a takePicture is in flight. Returns false if that shot has
    /// already ended. New X6 (C9) firmware sends the URI in this push
    /// instead of the takePicture completion; see the SDK sample's
    /// CameraConfigByJsonController.takePictureResponseNotification.
    /// The push carries no request ID, so it goes to the shot in flight.
    private var _photoWaiter: ((String, Double) -> Bool)?
    /// When the in-flight takePicture's completion arrived without a URI.
    private var _captureReplyTs: Double?
    /// The camera's photo list read just before the next takePicture
    /// (prepareShot). A shot whose URI comes in neither the reply nor
    /// the push finds its photo in a later list (shotPhoto).
    private var _photoListBeforeShot: Set<String>?
    /// syncCameraClock ran on this connection, whatever its result.
    /// Cleared when the connection drops. Under `lock`.
    private var _clockSyncTried = false
    /// setAppid and PureShot have been sent on this connection.
    /// Cleared when the connection drops, so a reconnect sends them again.
    /// Kept under `lock`, like the other state here: shots, SDK callbacks
    /// and the disconnect handler use it from different threads.
    private var _isPureShotSet = false
    private var isPureShotSet: Bool {
        get {
            lock.lock()
            defer { lock.unlock() }
            return _isPureShotSet
        }
        set {
            lock.lock()
            defer { lock.unlock() }
            _isPureShotSet = newValue
        }
    }
    /// The app's own photo mode settings were accepted on this connection
    /// (switchToPhotoMode). Cleared when the connection drops. Under
    /// `lock`, as isPureShotSet.
    private var _isPhotoModeSet = false
    private(set) var isPhotoModeSet: Bool {
        get {
            lock.lock()
            defer { lock.unlock() }
            return _isPhotoModeSet
        }
        set {
            lock.lock()
            defer { lock.unlock() }
            _isPhotoModeSet = newValue
        }
    }
    /// The camera settings run in progress (applyCameraSettings). A shot
    /// taken meanwhile waits for it instead of sending the settings twice.
    private var settingsTask: Task<Void, Never>?

    /// Shots captured but not yet downloaded.
    var pendingDownloadCount: Int {
        lock.lock()
        defer { lock.unlock() }
        return _pendingURIs.count
    }

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

    private var kvoToken: NSKeyValueObservation?
    private var disconnectToken: NSKeyValueObservation?
    private var photoPushObserver: NSObjectProtocol?
    private var takePictureStateObserver: NSObjectProtocol?

    init(config: CameraConfig) {
        self.id = config.id
        self.model = config.model
        self.serial = config.serial
        self.extrinsic = config.extrinsic
        self.maskPath = config.mask
    }

    deinit {
        stopPhotoPushObserver()
    }

    // MARK: - Connection

    // The heartbeat runs in Insta360ConnectionMonitor for as long as
    // the socket is connected, as in the SDK sample, not per instance.

    func connect() async -> Bool {
        onStatusUpdate?(
            "[\(id)] Waiting for SDK connection..."
        )
        startPhotoPushObserver()

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
        stopPhotoPushObserver()
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
        _photoWaiter = nil
        _captureInFlight = false
        lock.unlock()
        cont?.resume(returning: nil)
    }

    func reconnect(maxRetries: Int = 3) async -> Bool {
        // disconnect() clears the callbacks. Keep them, or this
        // instance logs nothing after Retry.
        let statusUpdate = onStatusUpdate
        let disconnectHandler = onDisconnect
        await disconnect()
        onStatusUpdate = statusUpdate
        onDisconnect = disconnectHandler
        for attempt in 1...maxRetries {
            // As in the original app. Insta360CaptureManager starts the
            // live preview afterwards, once the settings are in place.
            if await connect() {
                await applyCameraSettings()
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

    // syncTimeMsToCamera SETS the camera clock each call.
    // Sample 0 measures the original camera clock (pre-set).
    // Samples 1–N measure how accurately each set landed
    // (residual offset). We select the best by lowest RTT.
    func calibrateClockOffset() async -> ClockOffset {
        let wallSec = Date().timeIntervalSince1970
        let arkitSec = ProcessInfo.processInfo.systemUptime
        let fallback = ClockOffset(
            offsetMs: 0, uncertaintyMs: 999,
            sampleCount: 0, preSetSample: nil,
            samples: [],
            wallClockSec: wallSec,
            arkitTimeSec: arkitSec
        )
        onStatusUpdate?(
            "[\(id)] clockSync: start"
                + " \(kClockSyncTotalSamples) samples"
                + " (main=\(Thread.isMainThread))"
        )
        var preSet: ClockSyncSample?
        var postSetSamples: [ClockSyncSample] = []
        for i in 0..<kClockSyncTotalSamples {
            let sample = await singleSyncSample()
            if let s = sample {
                let tag = i == 0 ? " [pre-set]" : ""
                onStatusUpdate?(String(
                    format: "[\(id)] sync#%d:"
                        + " off=%.1fms rtt=%.0fms%@",
                    i, s.offsetMs, s.roundTripMs, tag
                ))
                if i == 0 {
                    preSet = s
                } else {
                    postSetSamples.append(s)
                }
            } else {
                onStatusUpdate?(
                    "[\(id)] sync#\(i): failed"
                )
            }
        }
        let wallSecEnd = Date().timeIntervalSince1970
        let arkitSecEnd = ProcessInfo.processInfo.systemUptime
        let avgWall = (wallSec + wallSecEnd) / 2
        let avgArkit = (arkitSec + arkitSecEnd) / 2
        guard !postSetSamples.isEmpty else {
            onStatusUpdate?(
                "[\(id)] clockSync: 0 valid samples"
            )
            clockOffset = fallback
            return fallback
        }
        let sorted = postSetSamples.sorted {
            $0.roundTripMs < $1.roundTripMs
        }
        let best = Array(sorted.prefix(kClockSyncBestCount))
        let avgOffset = best.map(\.offsetMs).reduce(
            0, +
        ) / Double(best.count)
        let maxRtt = best.map(\.roundTripMs).max() ?? 0
        let result = ClockOffset(
            offsetMs: avgOffset,
            uncertaintyMs: maxRtt / 2,
            sampleCount: postSetSamples.count,
            preSetSample: preSet,
            samples: postSetSamples,
            wallClockSec: avgWall,
            arkitTimeSec: avgArkit
        )
        onStatusUpdate?(String(
            format: "[\(id)] clockSync: done"
                + " res=%.1fms ±%.0fms (%d samples)",
            result.offsetMs, result.uncertaintyMs,
            postSetSamples.count
        ))
        clockOffset = result
        return result
    }

    private func singleSyncSample()
        async -> ClockSyncSample?
    {
        await withCheckedContinuation { cont in
            let done = NSLock()
            var didResume = false

            func finish(_ result: ClockSyncSample?) {
                done.lock()
                defer { done.unlock() }
                guard !didResume else { return }
                didResume = true
                cont.resume(returning: result)
            }

            DispatchQueue.main.async { [weak self] in
                self?.onStatusUpdate?(
                    "[\(self?.id ?? "?")] syncTime:"
                    + " sending…"
                )
                let sendMs = Date()
                    .timeIntervalSince1970 * 1000
                INSCameraManager.shared()
                    .commandManager
                    .syncTimeMsToCamera(
                        withTryCount: 1,
                        dTimeMsMax: 500
                    ) { dTimeMs, error in
                        let recvMs = Date()
                            .timeIntervalSince1970
                            * 1000
                        if let error {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")]"
                                + " syncTime err:"
                                + " \(error)"
                            )
                            finish(nil)
                        } else {
                            let midMs = (sendMs + recvMs)
                                / 2
                            let camMs = midMs
                                + Double(dTimeMs)
                            finish(
                                ClockSyncSample(
                                    phoneSendMs: sendMs,
                                    phoneRecvMs: recvMs,
                                    cameraMs: camMs,
                                    offsetMs: Double(
                                        dTimeMs
                                    )
                                )
                            )
                        }
                    }
            }

            DispatchQueue.main.asyncAfter(
                deadline: .now() + 5
            ) { [weak self] in
                done.lock()
                let alreadyDone = didResume
                done.unlock()
                if !alreadyDone {
                    self?.onStatusUpdate?(
                        "[\(self?.id ?? "?")]"
                        + " syncTime: timeout (5s)"
                    )
                }
                finish(nil)
            }
        }
    }

    // MARK: - Camera clock

    /// Sets the camera's clock to the phone's, once per connection. The
    /// camera names photos by its clock and the photo-list fallback
    /// compares those names (shotPhoto); a camera whose clock was never
    /// set names new photos with an old date. prepareShot calls this
    /// right after setAppid + PureShot: the sample says the camera takes
    /// settings only after setAppid, and sent at connect, before it, the
    /// X5 answered error 400. Logs the camera's time and time zone before
    /// and after where the camera reports them (the X5 doesn't). The time
    /// zone is left as it is.
    private func syncCameraClock() async {
        lock.lock()
        let tried = _clockSyncTried
        _clockSyncTried = true
        lock.unlock()
        guard !tried else { return }
        let before = await readCameraClock()
        onStatusUpdate?("[\(id)] camera clock: \(before)")
        guard let reportedMs = await setCameraClock() else { return }
        let after = await readCameraClock()
        onStatusUpdate?(
            "[\(id)] camera clock set from the phone"
            + " (SDK reports \(reportedMs) ms): \(after)"
        )
    }

    /// syncTimeMsToCamera as calibrateClockOffset sends it; it sets the
    /// camera's clock. Returns the time difference the SDK reports, in
    /// ms, or nil if it failed.
    private func setCameraClock() async -> UInt? {
        await withCheckedContinuation {
            (cont: CheckedContinuation<UInt?, Never>) in
            let done = NSLock()
            var finished = false

            // Ends once. A failure is logged before the caller resumes,
            // so it stays in order in the log.
            func finish(_ value: UInt?, failure: String? = nil) {
                done.lock()
                let first = !finished
                finished = true
                done.unlock()
                guard first else { return }
                if let failure {
                    self.onStatusUpdate?(
                        "[\(self.id)] camera clock not set: \(failure)")
                }
                cont.resume(returning: value)
            }

            DispatchQueue.main.async {
                INSCameraManager.shared().commandManager
                    .syncTimeMsToCamera(
                        withTryCount: 1, dTimeMsMax: 500
                    ) { dTimeMs, error in
                        if let error {
                            finish(nil, failure: "\(error)")
                        } else {
                            finish(dTimeMs)
                        }
                    }
            }

            DispatchQueue.main.asyncAfter(deadline: .now() + 5) {
                finish(nil, failure: "no reply in 5 s")
            }
        }
    }

    /// The camera's clock and time zone as text for the log.
    private func readCameraClock() async -> String {
        await withCheckedContinuation {
            (cont: CheckedContinuation<String, Never>) in
            let done = NSLock()
            var finished = false

            func finish(_ value: String) {
                done.lock()
                let first = !finished
                finished = true
                done.unlock()
                if first { cont.resume(returning: value) }
            }

            DispatchQueue.main.async {
                let types: [NSNumber] = [
                    NSNumber(
                        value: INSCameraOptionsType.localTime.rawValue
                    ),
                    NSNumber(
                        value: INSCameraOptionsType.timeZone.rawValue
                    ),
                ]
                INSCameraManager.shared().commandManager
                    .getOptionsWithTypes(types) { error, options, _ in
                        guard let options else {
                            finish(
                                "read failed: "
                                + (error.map { "\($0)" } ?? "no options")
                            )
                            return
                        }
                        finish(Self.clockText(
                            localTime: options.localTime,
                            zoneSeconds: options.timeZoneSecondsFromGMT))
                    }
            }

            DispatchQueue.main.asyncAfter(deadline: .now() + 5) {
                finish("read timeout")
            }
        }
    }

    private static let clockFormatter: DateFormatter = {
        let fmt = DateFormatter()
        fmt.dateFormat = "yyyy-MM-dd HH:mm:ss"
        fmt.locale = Locale(identifier: "en_US_POSIX")
        fmt.timeZone = TimeZone(identifier: "UTC")
        return fmt
    }()

    /// INSCameraOptions.localTime, read as seconds since 1970 (or
    /// milliseconds if it is that large), next to the phone's time, and
    /// timeZoneSecondsFromGMT as an offset.
    private static func clockText(
        localTime: UInt64, zoneSeconds: Int
    ) -> String {
        func two(_ v: UInt) -> String { v < 10 ? "0\(v)" : "\(v)" }
        let a = zoneSeconds.magnitude
        let sign = zoneSeconds < 0 ? "-" : "+"
        let zone = "UTC\(sign)\(two(a / 3600)):\(two(a % 3600 / 60))"
        let phone = clockFormatter.string(from: Date())
        guard localTime > 0 else {
            return "not reported (localTime 0, time zone \(zone));"
                + " phone \(phone) UTC"
        }
        // Seconds would put 10^11 after the year 5000.
        let sec = localTime > 100_000_000_000
            ? Double(localTime) / 1000 : Double(localTime)
        let cam = clockFormatter.string(
            from: Date(timeIntervalSince1970: sec))
        return "localTime \(localTime) = \(cam) UTC"
            + " (phone \(phone) UTC), time zone \(zone)"
    }

    // MARK: - Capture

    // A shot follows the SDK sample's take-picture page
    // (CameraConfigByJsonController.swift, opened with "Record new"):
    // "X5 Take Picture" sends setAppid, then PureShot, then takePicture,
    // and the page's live preview runs (here on the X6 only; see
    // Insta360CaptureManager.startLivePreview). On top of that, the camera
    // has the original app's own settings (applyCameraSettings) and
    // takePicture carries the original app's exposure value
    // (takePictureOptions); the sample sends neither. Like the sample, no
    // warmup photo is taken.

    /// Everything a shot does before takePicture. Calibration calls this
    /// first and then fires with skipPreChecks, so takePicture follows
    /// the ARKit frame closely.
    func prepareForCapture() async -> Bool {
        guard await ensureSDKConnected() else {
            return false
        }
        await prepareShot()
        return true
    }

    /// Takes one photo. `onCommand` runs on the main actor with the
    /// systemUptime of takePicture, just before the command is sent, so a
    /// caller can time what follows the shot from the command itself
    /// rather than from the start of prepareShot. It isn't called when
    /// the shot ends before takePicture.
    func capture(
        arkitTimestamp: Double,
        scanIndex: Int,
        timeout: Double = 45,
        skipPreChecks: Bool = false,
        onCommand: (@MainActor @Sendable (Double) -> Void)? = nil
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
            await prepareShot()
        }

        lock.lock()
        let photoListBefore = _photoListBeforeShot
        _photoListBeforeShot = nil
        lock.unlock()

        // One takePicture per shot, as the sample sends one per press.
        // The old retry after a missed reply took a second photo.
        let commandTs = ProcessInfo.processInfo.systemUptime
        // Awaited, so the caller's windows have started before the
        // command goes out and before this capture returns.
        if let onCommand {
            await onCommand(commandTs)
        }
        let captureResult = await attemptTakePicture(
            timeout: timeout,
            photoListBefore: photoListBefore
        )

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

    /// Brings the camera to where the sample's page is when its button
    /// sends takePicture: setAppid and PureShot have been sent, and on the
    /// X6 the live preview runs.
    private func prepareShot() async {
        // The original app's camera settings: waits for them if they're
        // still being sent at connect, and resends them if they failed.
        await ensurePhotoMode()
        // Doesn't wait for the preview to restart after the last shot:
        // the stream restarts on its own, and waiting delayed every shot.
        // The sample keeps it running while you take pictures. Only the
        // X6 runs a preview, so this logs nothing for other cameras.
        let (previewOn, previewRunning) = await MainActor.run {
            (
                Insta360LivePreview.shared.isActive,
                Insta360LivePreview.shared.isRunning
            )
        }
        if previewOn, !previewRunning {
            onStatusUpdate?(
                "[\(id)] live preview restarting — not waiting for it"
            )
        }
        // setAppid and PureShot once per connection, not before every
        // shot as the sample does. Sent again only if they failed.
        if !isPureShotSet {
            isPureShotSet = await setAppIdAndPureShot()
            if !isPureShotSet {
                // The sample's plain "Take Picture" button sends
                // takePicture without them.
                onStatusUpdate?(
                    "[\(id)] taking the photo without setAppid/PureShot"
                )
            }
        }
        // Once per connection, after setAppid (see syncCameraClock).
        await syncCameraClock()
        // Read last, so the list is as close to the shot as possible.
        let list = await fetchPhotoURIs()
        lock.lock()
        _photoListBeforeShot = list
        lock.unlock()
        // The newest name shows the camera clock's date in the log.
        let count = list.map { "\($0.count) photos" } ?? "read failed"
        var dated = ""
        if let list {
            let (newest, future) = Self.newestFile(
                list, notAfter: Self.latestPlausibleStamp())
            if let newest {
                dated += ", newest "
                    + (newest.uri as NSString).lastPathComponent
            }
            if !future.isEmpty {
                let names = future.prefix(3).map {
                    ($0 as NSString).lastPathComponent
                }.joined(separator: ", ")
                let more = future.count > 3
                    ? " and \(future.count - 3) more" : ""
                dated += "; dated after today, ignored: \(names)\(more)"
            }
        }
        onStatusUpdate?(
            "[\(id)] photo list before shot: \(count)\(dated)"
        )
    }

    // MARK: - The original app's camera settings

    // Restored from the original app (Eecornwell/atlas-scanner, mobile
    // branch, CameraInstance.swift): single photo mode on both sensors,
    // then FlowState and horizon leveling off. In-camera stitching off
    // comes last. Insta360CaptureManager sends them when the camera
    // connects, before the live preview starts, so the preview uses the
    // sensors they set. Before a shot they're sent again only if they
    // failed (ensurePhotoMode), as in the original.

    /// Sends switchToPhotoMode, disableStabilization, then
    /// disableInCameraStitching. A call while a run is in progress waits
    /// for that run instead of starting another.
    func applyCameraSettings() async {
        lock.lock()
        let task: Task<Void, Never>
        if let running = settingsTask {
            task = running
        } else {
            task = Task { [weak self] in
                await self?.switchToPhotoMode()
                await self?.disableStabilization()
                await self?.disableInCameraStitching()
            }
            settingsTask = task
        }
        lock.unlock()
        await task.value
        lock.lock()
        if settingsTask == task { settingsTask = nil }
        lock.unlock()
    }

    /// Before a shot: waits for a settings run still in progress, or, if
    /// the photo mode settings failed on this connection, checks that the
    /// camera answers and sends them again.
    func ensurePhotoMode() async {
        lock.lock()
        let running = settingsTask
        lock.unlock()
        if let running {
            await running.value
            return
        }
        guard !isPhotoModeSet else { return }
        let ready = await warmUpCommandChannel()
        guard ready else { return }
        await applyCameraSettings()
    }

    /// Whether both sensors are already active, so switchToPhotoMode can
    /// skip activating them.
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

    /// Single photo mode (photo sub mode 0) with both sensors focused, the
    /// default output type, and both sensors active. The original also
    /// sent the X6 wake call setAppAccessFileState(.liveView) first. It's
    /// left out: Insta360ConnectionMonitor sends it on every connection,
    /// and the API doc says the same call locks the X6's screen while the
    /// live preview stream runs, as it can when ensurePhotoMode resends
    /// these settings.
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
                // A normal 360 photo is photo sub mode 0
                // (INSPhotoSubModeSingle), as the desktop capture sets
                // (PHOTO_SINGLE). The 6 used before is the function mode
                // of a normal photo in the SDK sample's mode table; as a
                // photo sub mode it means InstaPano HDR, a mode the X6
                // doesn't have.
                opts.photoSubMode = 0
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

    /// Turns off FlowState (in-camera stabilization) and horizon leveling.
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

    /// Turns in-camera stitching off, so photos stay dual-fisheye INSP.
    /// With it on, the X5 saves stitched JPGs instead (Insta360's X5 file
    /// format docs), and the export treats any 2:1 image as dual fisheye.
    /// The X6 live preview needs it off too: the preview stitches in the
    /// app, and the SDK sample's comment says the seams look wrong and
    /// parts of the image repeat otherwise (disableInternalSplicingForPreview).
    /// Sent for every camera, so it no longer depends on the preview.
    func disableInCameraStitching() async {
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
                opts.enableInternalSplicing = false
                let types: [NSNumber] = [
                    NSNumber(
                        value: INSCameraOptionsType
                            .internalSplicing.rawValue
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
                                + "in-camera stitching off err: "
                                + "\(error)"
                            )
                        } else {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "in-camera stitching off"
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

    // MARK: - Storage and photo list (API doc, X6 dual storage)

    /// The X6 (SDK name C9) has internal storage and an SD card. The API
    /// doc reads them separately and lists files with
    /// hasInternalAndSDStorage = true.
    private static func isDualStorageCamera() -> Bool {
        INSCameraManager.shared().currentCamera?.name
            == kInsta360CameraNameC9
    }

    private static func cardStateText(
        _ state: INSCameraCardState
    ) -> String {
        switch state {
        case .normal: return "Normal"
        case .noCard: return "NoCard"
        case .noSpace: return "NoSpace"
        case .invalidFormat: return "InvalidFormat"
        case .writeProtectCard: return "WriteProtectCard"
        case .unknownError: return "UnknownError"
        @unknown default: return "state \(state.rawValue)"
        }
    }

    private static func locationText(
        _ location: INSStorageCardLocation
    ) -> String {
        switch location {
        case .camera: return "camera"
        case .reader: return "reader"
        case .inner: return "internal"
        case .SD: return "SD card"
        @unknown default: return "location \(location.rawValue)"
        }
    }

    private static func storageText(
        _ status: INSCameraStorageStatus?
    ) -> String {
        guard let s = status else { return "none" }
        let gb = Double(1024 * 1024 * 1024)
        return Self.cardStateText(s.cardState)
            + String(
                format: " %.1f of %.1f GB free",
                Double(s.freeSpace) / gb, Double(s.totalSpace) / gb
            )
    }

    /// Logs the storage the camera captures to, as the API doc's
    /// "X6 special handling" reads it: the SD card is the
    /// cardLocation == .SD item of storageStateList, internal storage is
    /// cameraStorageStatus. storageStatus is the main storage, the one
    /// photos are written to (sample comment, RootViewController+Sections).
    func logStorageStatus() async {
        let text = await withCheckedContinuation {
            (cont: CheckedContinuation<String, Never>) in
            let done = NSLock()
            var finished = false

            func finish(_ value: String) {
                done.lock()
                let first = !finished
                finished = true
                done.unlock()
                if first { cont.resume(returning: value) }
            }

            DispatchQueue.main.async {
                let dual = Self.isDualStorageCamera()
                let types: [NSNumber] = [
                    NSNumber(
                        value: INSCameraOptionsType.storageState.rawValue
                    ),
                    NSNumber(
                        value: INSCameraOptionsType
                            .storageStateList.rawValue
                    ),
                ]
                INSCameraManager.shared().commandManager
                    .getOptionsWithTypes(types) { error, options, _ in
                        guard let options else {
                            finish(
                                "storage read failed: "
                                + (error.map { "\($0)" } ?? "no options")
                            )
                            return
                        }
                        let list = options.storageStateList
                            as? [INSCameraStorageStatus] ?? []
                        let main = options.storageStatus.map {
                            Self.locationText($0.cardLocation)
                        } ?? "?"
                        if dual {
                            let sd = list.first { $0.cardLocation == .SD }
                            let inner = options.cameraStorageStatus
                                ?? list.first { $0.cardLocation == .inner }
                            finish(
                                "storage: photos go to \(main);"
                                + " internal \(Self.storageText(inner));"
                                + " SD card \(Self.storageText(sd))"
                            )
                        } else {
                            finish(
                                "storage: SD card "
                                + Self.storageText(options.storageStatus)
                            )
                        }
                    }
            }

            DispatchQueue.main.asyncAfter(deadline: .now() + 10) {
                finish("storage read timeout")
            }
        }
        onStatusUpdate?("[\(id)] \(text)")
    }

    /// URIs of every photo on the camera, from fetchPhotoList as the API
    /// doc's "Get File List" does it. On the X6 the list covers both
    /// storages: SD card photos are in sdResources (/DCIM/Camera01/...),
    /// internal ones in cameraResources (/storage_internal/DCIM/...).
    /// Nil if the list can't be read.
    private func fetchPhotoURIs() async -> Set<String>? {
        await withCheckedContinuation {
            (cont: CheckedContinuation<Set<String>?, Never>) in
            let done = NSLock()
            var finished = false

            func finish(_ value: Set<String>?) {
                done.lock()
                let first = !finished
                finished = true
                done.unlock()
                if first { cont.resume(returning: value) }
            }

            DispatchQueue.main.async { [weak self] in
                let dual = Self.isDualStorageCamera()
                let options = INSGetFileListOptions()
                options.type = .sdAndCamera
                options.hasInternalAndSDStorage = dual
                INSCameraManager.shared().commandManager.fetchPhotoList(
                    with: options
                ) { error, res in
                    var list: [INSCameraBaseFileInfo] = []
                    if dual {
                        if let sd = res?.sdResources {
                            list.append(contentsOf: sd)
                        }
                        if let cam = res?.cameraResources {
                            list.append(contentsOf: cam)
                        }
                    } else {
                        list = res?.cameraResources ?? []
                    }
                    if list.isEmpty, let error {
                        self?.onStatusUpdate?(
                            "[\(self?.id ?? "?")] photo list: \(error)"
                        )
                        finish(nil)
                        return
                    }
                    finish(Set(list.map { $0.uri }.filter { !$0.isEmpty }))
                }
            }

            DispatchQueue.main.asyncAfter(deadline: .now() + 15) {
                finish(nil)
            }
        }
    }

    /// Date and time (YYYYMMDDHHMMSS) and sequence number from a camera
    /// file name, compared in that order.
    private struct FileStamp: Comparable {
        let time: Int64
        let seq: Int

        static func < (a: FileStamp, b: FileStamp) -> Bool {
            a.time != b.time ? a.time < b.time : a.seq < b.seq
        }
    }

    /// The date, time and sequence number in a camera file name such as
    /// IMG_20261115_135444_00_829.insp: prefix, YYYYMMDD, HHMMSS, lens
    /// field, sequence number. The date and time are the camera clock's.
    /// Nil for a name in another form.
    private static func fileStamp(_ uri: String) -> FileStamp? {
        let name = ((uri as NSString).lastPathComponent as NSString)
            .deletingPathExtension
        let f = name.split(separator: "_", omittingEmptySubsequences: false)
        guard f.count >= 5, f[1].count == 8, f[2].count == 6,
              let date = Int64(f[1]), let time = Int64(f[2]),
              let seq = Int(f[4])
        else { return nil }
        return FileStamp(time: date * 1_000_000 + time, seq: seq)
    }

    /// The latest date a photo's name can honestly have: the phone's
    /// local time plus a day (camera names carry the camera's local time;
    /// the day covers a camera set to another time zone). A later name
    /// was written while the camera's clock was wrong: an X5 card held
    /// IMG_20851108_185409_00_279.insp, which every new photo would
    /// otherwise have to be newer than.
    private static func latestPlausibleStamp() -> FileStamp {
        let text = stampFormatter.string(
            from: Date().addingTimeInterval(86_400))
        return FileStamp(time: Int64(text) ?? .max, seq: .max)
    }

    private static let stampFormatter: DateFormatter = {
        let fmt = DateFormatter()
        fmt.locale = Locale(identifier: "en_US_POSIX")
        fmt.calendar = Calendar(identifier: .gregorian)
        fmt.timeZone = .current
        fmt.dateFormat = "yyyyMMddHHmmss"
        return fmt
    }()

    /// The file in a photo list with the latest fileStamp up to `limit`
    /// (latestPlausibleStamp), or nil if no name has one, and the names
    /// dated after `limit`, sorted. Of equal stamps the larger URI wins,
    /// so the result doesn't depend on the set's order.
    private static func newestFile(
        _ uris: Set<String>, notAfter limit: FileStamp
    ) -> (newest: (uri: String, stamp: FileStamp)?, future: [String]) {
        var best: (uri: String, stamp: FileStamp)?
        var future: [String] = []
        for u in uris {
            guard let s = fileStamp(u) else { continue }
            if s > limit {
                future.append(u)
                continue
            }
            if let b = best, s < b.stamp || (s == b.stamp && u < b.uri) {
                continue
            }
            best = (u, s)
        }
        return (best, future.sorted())
    }

    /// The shot's photo among `added`, the files that weren't in the
    /// list before the shot. Only an INSP counts, and only one whose
    /// name's date, time and sequence number are later than
    /// `newestBefore`, the newest file that list had, and not after
    /// `limit`; of several, the latest. That keeps an older photo out if
    /// the list before the shot missed some files. If no INSP qualifies
    /// but exactly one is new, it still counts, with a note: one new INSP
    /// means the list before was complete, so it is the shot, and the
    /// camera's clock is off (never set, for example). Nil while nothing
    /// qualifies; `note` says why, for the log.
    private static func shotPhoto(
        added: Set<String>,
        newestBefore: (uri: String, stamp: FileStamp)?,
        notAfter limit: FileStamp
    ) -> (uri: String?, note: String?) {
        let insp = added.filter(isINSP)
        guard !insp.isEmpty else { return (nil, nil) }
        var later: [(uri: String, stamp: FileStamp)] = []
        for u in insp {
            guard let s = fileStamp(u), s <= limit else { continue }
            if let b = newestBefore, s <= b.stamp { continue }
            later.append((u, s))
        }
        let newest = newestBefore.map {
            ($0.uri as NSString).lastPathComponent
        } ?? "no dated file"
        if let pick = later.max(by: { $0.stamp < $1.stamp }) {
            let note = later.count > 1
                ? "\(later.count) new INSPs after \(newest), took the latest"
                : nil
            return (pick.uri, note)
        }
        if insp.count == 1, let only = insp.first {
            let note: String
            if let s = fileStamp(only) {
                note = s > limit
                    ? "dated after today; is the camera clock ahead?"
                    : "not dated after \(newest), the newest file already"
                        + " on the camera; is the camera clock behind?"
            } else {
                note = "its name has no date and time"
            }
            return (only, note)
        }
        return (nil, "\(insp.count) new INSPs, none dated after \(newest)")
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
                                + " command check: "
                                + "\(error)"
                            )
                        } else {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")]"
                                + " command check OK"
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
                        + " command check timeout"
                        + " (\(Int(timeout))s)"
                    )
                }
                finish(false)
            }
        }
    }

    /// Sends what the SDK sample's "X5 Take Picture" sends before every
    /// takePicture (CameraConfigByJsonController.swift): setAppid, then
    /// raw capture type PureShot (3) for function mode 8 (HDR_IMAGE),
    /// option type 25. The sample's comment: the camera accepts photo
    /// settings only after setAppid, and without it the photo still
    /// includes a DNG. Returns whether both succeeded.
    private func setAppIdAndPureShot() async -> Bool {
        await withCheckedContinuation {
            (continuation: CheckedContinuation<Bool, Never>)
                in
            let resumed = NSLock()
            var didResume = false

            func finish(_ ok: Bool) {
                resumed.lock()
                defer { resumed.unlock() }
                guard !didResume else { return }
                didResume = true
                continuation.resume(returning: ok)
            }

            DispatchQueue.main.async { [weak self] in
                let commands = INSCameraManager.shared()
                    .commandManager
                commands.setAppidCompletion { [weak self] error in
                    if let error {
                        self?.onStatusUpdate?(
                            "[\(self?.id ?? "?")] "
                            + "setAppid err: \(error)"
                        )
                        finish(false)
                        return
                    }
                    let photo = INSPhotographyOptions()
                    photo.rawCaptureType = UInt8(
                        INSCameraRawCaptureType.pureshot.rawValue
                    )
                    let types: [NSNumber] = [
                        NSNumber(
                            value: INSPhotographyOptionsType
                                .rawCaptureType.rawValue
                        ),
                    ]
                    commands.setPhotographyOptions(
                        photo,
                        for: INSCameraFunctionMode(functionMode: 8),
                        types: types
                    ) { [weak self] error, _ in
                        if let error {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "PureShot err: \(error)"
                            )
                            finish(false)
                        } else {
                            self?.onStatusUpdate?(
                                "[\(self?.id ?? "?")] "
                                + "setAppid + PureShot set"
                            )
                            finish(true)
                        }
                    }
                }
            }

            DispatchQueue.main.asyncAfter(
                deadline: .now() + 15
            ) { [weak self] in
                resumed.lock()
                let alreadyDone = didResume
                resumed.unlock()
                if !alreadyDone {
                    self?.onStatusUpdate?(
                        "[\(self?.id ?? "?")] "
                        + "setAppid/PureShot timeout"
                    )
                }
                finish(false)
            }
        }
    }

    private typealias CaptureAck = (
        uri: String, callbackTimestamp: Double
    )

    /// takePicture options. Raw capture type PureShot and countDown 0 are
    /// what the SDK sample's "X5 Take Picture" sets. Mode .normal and the
    /// exposure value are the original app's (commit 9007db5): program 3
    /// with a 120 s shutter in currenExposureOptions. The SDK header says
    /// that field is "used to calculate timeout": it tells the SDK how long
    /// the shot can take, so the SDK waits long enough for the reply. It
    /// doesn't set the camera's exposure; that is setPhotographyOptions
    /// with stillExposure (API doc, Exposure Mode), which isn't sent.
    private static func takePictureOptions() -> INSTakePictureOptions {
        let opts = INSTakePictureOptions()
        opts.mode = .normal
        opts.rawCaptureType = .pureshot
        opts.countDown = 0
        let exp = INSCameraExposureOptions()
        exp.program = 3
        exp.shutterSpeed = CMTime(
            value: 120, timescale: 1
        )
        opts.currenExposureOptions = exp
        return opts
    }

    /// Sends one takePicture and waits for its photo URI. Old firmware
    /// returns the URI in the takePicture completion. New X6 (C9)
    /// firmware returns it in an INSCameraTakePictureResponse push
    /// instead (handlePhotoPush); the API doc and SDK sample handle both.
    /// If the reply has no URI, this also checks the camera's photo list
    /// for the new photo (findNewPhoto), so a push that never comes
    /// doesn't lose the shot. `photoListBefore` is the list before the
    /// shot; without it only the reply and the push count.
    private func attemptTakePicture(
        timeout: Double,
        photoListBefore: Set<String>? = nil
    ) async -> CaptureAck? {
        return await withTaskCancellationHandler {
            await withCheckedContinuation {
                continuation in
                self.lock.lock()
                self._captureGen += 1
                let gen = self._captureGen
                self._captureCont = continuation
                self._captureReplyTs = nil
                self.lock.unlock()

                // Ends this attempt once. Returns false if it had
                // already ended (timeout, cancel, clearCaptureState).
                func finish(_ ack: CaptureAck?) -> Bool {
                    self.lock.lock()
                    guard gen == self._captureGen,
                          let cont = self._captureCont
                    else {
                        self.lock.unlock()
                        return false
                    }
                    self._captureCont = nil
                    self._photoWaiter = nil
                    self.lock.unlock()
                    cont.resume(returning: ack)
                    return true
                }

                // The push's time stands in for the reply time only if
                // the completion hasn't arrived yet.
                self.lock.lock()
                self._photoWaiter = { uri, pushTs in
                    self.lock.lock()
                    let replyTs = self._captureReplyTs
                    self.lock.unlock()
                    return finish(
                        CaptureAck(
                            uri: uri,
                            callbackTimestamp: replyTs ?? pushTs
                        )
                    )
                }
                self.lock.unlock()

                DispatchQueue.main.async {
                    INSCameraManager.shared()
                        .commandManager
                        .takePicture(
                            with: Self.takePictureOptions()
                        ) { error, photoInfo in
                            let cbTs = ProcessInfo
                                .processInfo.systemUptime
                            // Only an INSP is the shot's photo. A reply
                            // with just a DNG waits like one without a URI.
                            let (uri, others) = Self.photoURI(photoInfo)
                            let hdr = photoInfo?.hdrUris?.count ?? 0
                            let burst = photoInfo?.burstUris?.count
                                ?? 0
                            var note = "[\(self.id)] cb:"
                                + " gen=\(gen)"
                                + " uri=\(uri ?? "nil")"
                            if hdr > 0 { note += " hdr=\(hdr)" }
                            if burst > 0 { note += " burst=\(burst)" }
                            if !others.isEmpty {
                                note += " not INSP, ignored: "
                                    + others.joined(separator: ", ")
                            }
                            if let error {
                                note += " err=\(error)"
                            } else if uri == nil {
                                note += " (waiting for photo push)"
                            }
                            self.onStatusUpdate?(note)
                            if error != nil {
                                _ = finish(nil)
                            } else if let uri {
                                _ = finish(
                                    CaptureAck(
                                        uri: uri,
                                        callbackTimestamp: cbTs
                                    )
                                )
                            } else {
                                // No URI and no error: the URI comes in
                                // a push, or the photo shows up in the
                                // photo list. Wait for either until the
                                // timeout.
                                self.lock.lock()
                                if gen == self._captureGen {
                                    self._captureReplyTs = cbTs
                                }
                                self.lock.unlock()
                                if let before = photoListBefore {
                                    Task { [weak self] in
                                        await self?.findNewPhoto(
                                            gen: gen,
                                            before: before,
                                            replyTs: cbTs,
                                            finish: finish
                                        )
                                    }
                                }
                            }
                        }
                }

                DispatchQueue.main.asyncAfter(
                    deadline: .now() + timeout
                ) {
                    if finish(nil) {
                        self.onStatusUpdate?(
                            "[\(self.id)] takePicture"
                            + " timeout"
                            + " (\(Int(timeout))s)"
                        )
                    }
                }
            }
        } onCancel: {
            self.lock.lock()
            self._captureGen += 1
            let cont = self._captureCont
            self._captureCont = nil
            self._photoWaiter = nil
            self.lock.unlock()
            cont?.resume(returning: nil)
        }
    }

    /// Reads the photo list every second until it has the shot's photo
    /// (shotPhoto), then ends the shot with it. Stops when the shot ends
    /// another way (push, timeout, cancel) or a newer shot starts.
    private func findNewPhoto(
        gen: UInt64,
        before: Set<String>,
        replyTs: Double,
        finish: @escaping (CaptureAck?) -> Bool
    ) async {
        let limit = Self.latestPlausibleStamp()
        let newestBefore = Self.newestFile(before, notAfter: limit).newest
        var reads = 0
        var ignored = Set<String>()
        var lastNote: String?
        while true {
            // 1 s, not 2: in the logs the photo appeared on the 2nd or
            // 3rd read, so a shorter interval finds it sooner.
            try? await Task.sleep(for: .seconds(1))
            // Is this shot still waiting?
            lock.lock()
            let waiting = gen == _captureGen && _captureCont != nil
            lock.unlock()
            guard waiting else { return }

            reads += 1
            guard let now = await fetchPhotoURIs() else { continue }
            let added = now.subtracting(before)
            // A DNG (RAW on) or any other new file isn't the photo.
            let others = added.filter { !Self.isINSP($0) }
                .subtracting(ignored)
            if !others.isEmpty {
                ignored.formUnion(others)
                let names = others.sorted().prefix(5)
                    .joined(separator: ", ")
                let more = others.count > 5
                    ? " and \(others.count - 5) more" : ""
                onStatusUpdate?(
                    "[\(id)] new files that aren't INSP, ignored:"
                    + " \(names)\(more)"
                )
            }
            let (found, note) = Self.shotPhoto(
                added: added, newestBefore: newestBefore, notAfter: limit)
            guard let uri = found else {
                if let note, note != lastNote {
                    lastNote = note
                    onStatusUpdate?("[\(id)] photo list: \(note)")
                }
                continue
            }
            if finish(CaptureAck(uri: uri, callbackTimestamp: replyTs)) {
                onStatusUpdate?(
                    "[\(id)] new photo in photo list (read \(reads)):"
                    + " \(uri)"
                    + (note.map { " (\($0))" } ?? "")
                )
            }
            return
        }
    }

    /// The shot's photo in a takePicture reply or photo push: the first
    /// INSP among `uri`, `hdrUris` and `burstUris`. A single shot fills
    /// `uri`; HDR or AEB shots fill `hdrUris` and bursts fill
    /// `burstUris`, which the SDK sample also reads. The SDK documents
    /// HDR order as [EV0, -EV, +EV], so the first HDR URI is the normal
    /// exposure. `others` are the URIs that aren't INSP, for the log.
    private static func photoURI(
        _ info: INSCameraPhotoInfo?
    ) -> (uri: String?, others: [String]) {
        var all: [String] = []
        if let u = info?.uri { all.append(u) }
        all += info?.hdrUris ?? []
        all += info?.burstUris ?? []
        var others: [String] = []
        for u in all where !u.isEmpty && !isINSP(u) && !others.contains(u) {
            others.append(u)
        }
        return (all.first(where: isINSP), others)
    }

    /// Whether a camera file is an INSP, the 360 photo. Nothing else the
    /// camera writes for a shot is the shot's photo: with RAW on
    /// (PureShot not applied) it also writes a DNG.
    private static func isINSP(_ uri: String) -> Bool {
        (uri as NSString).pathExtension.lowercased() == "insp"
    }

    // MARK: - Photo URI push

    /// Observes on NotificationCenter.default, as the SDK sample does.
    /// Also logs INSCameraTakePictureStateUpdate, which the sample's root
    /// page observes, to show how far a shot got if no URI arrives.
    private func startPhotoPushObserver() {
        if photoPushObserver == nil {
            photoPushObserver = NotificationCenter.default.addObserver(
                forName: .INSCameraTakePictureResponse,
                object: nil,
                queue: nil
            ) { [weak self] note in
                self?.handlePhotoPush(note)
            }
        }
        if takePictureStateObserver == nil {
            takePictureStateObserver = NotificationCenter.default
                .addObserver(
                    forName: .INSCameraTakePictureStateUpdate,
                    object: nil,
                    queue: nil
                ) { [weak self] note in
                    let state = note.userInfo?["state"]
                        .map { "\($0)" } ?? "?"
                    self?.onStatusUpdate?(
                        "[\(self?.id ?? "?")] take picture state: \(state)"
                    )
                }
        }
    }

    private func stopPhotoPushObserver() {
        for token in [photoPushObserver, takePictureStateObserver] {
            if let token {
                NotificationCenter.default.removeObserver(token)
            }
        }
        photoPushObserver = nil
        takePictureStateObserver = nil
    }

    /// userInfo["photo"] is an INSCameraPhotoInfo. The push goes to the
    /// takePicture in flight; with none in flight it is ignored (for
    /// example a photo taken with the camera's own button). Only an INSP
    /// in it counts (photoURI).
    private func handlePhotoPush(_ note: Notification) {
        let info = note.userInfo?["photo"] as? INSCameraPhotoInfo
        let (insp, others) = Self.photoURI(info)
        let ignored = others.isEmpty ? ""
            : " (not INSP, ignored: " + others.joined(separator: ", ")
                + ")"
        guard let uri = insp else {
            onStatusUpdate?("[\(id)] photo push without an INSP" + ignored)
            return
        }
        lock.lock()
        let waiter = _photoWaiter
        lock.unlock()
        let used = waiter?(
            uri, ProcessInfo.processInfo.systemUptime
        ) ?? false
        onStatusUpdate?(
            "[\(id)] photo push: uri=\(uri)"
            + (used ? "" : " (no shot waiting, ignored)") + ignored
        )
    }

    // MARK: - Download

    func downloadPending(
        into directory: URL
    ) async -> [Insta360DownloadResult] {
        // Take the queue only while connected; otherwise leave it for a
        // later pass instead of dropping the shots.
        lock.lock()
        let connected = _isConnected
        let toDownload = connected ? _pendingURIs : []
        if connected { _pendingURIs.removeAll() }
        lock.unlock()

        guard !toDownload.isEmpty else {
            return []
        }

        var results: [Insta360DownloadResult] = []
        var failed: [(scanIndex: Int, uri: String)] = []
        for item in toDownload {
            if let r = await downloadOne(
                scanIndex: item.scanIndex,
                uri: item.uri,
                into: directory
            ) {
                results.append(r)
            } else {
                failed.append(item)
            }
        }

        // Put failures back so the next pass (the next shot, or the
        // retry at session end) tries them again.
        if !failed.isEmpty {
            var requeued = 0
            lock.lock()
            for item in failed {
                let n = (_downloadAttempts[item.uri] ?? 0) + 1
                _downloadAttempts[item.uri] = n
                if n < Self.maxDownloadAttempts {
                    _pendingURIs.append(item)
                    requeued += 1
                }
            }
            lock.unlock()
            onStatusUpdate?(
                "[\(id)] \(failed.count) download(s) failed,"
                + " \(requeued) re-queued"
            )
        }
        return results
    }

    // MARK: - Private

    private func resumeAllContinuations(
        clearURIs: Bool
    ) {
        let fallback = ClockOffset(
            offsetMs: 0, uncertaintyMs: 999,
            sampleCount: 0, preSetSample: nil,
            samples: [],
            wallClockSec: Date().timeIntervalSince1970,
            arkitTimeSec: ProcessInfo.processInfo
                .systemUptime
        )

        lock.lock()
        _isConnected = false
        _captureInFlight = false
        _photoWaiter = nil
        _captureReplyTs = nil
        if clearURIs {
            _pendingURIs.removeAll()
            _downloadAttempts.removeAll()
        }
        let connectCont = _connectCont
        let clockCont = _clockCont
        let captureCont = _captureCont
        let downloadCont = _downloadCont
        _connectCont = nil
        _clockCont = nil
        _captureCont = nil
        _downloadCont = nil
        // A new connection needs the camera settings, setAppid,
        // PureShot and the clock again.
        _isPureShotSet = false
        _isPhotoModeSet = false
        _clockSyncTried = false
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
        let started = ProcessInfo.processInfo.systemUptime
        return await withCheckedContinuation {
            continuation in
            lock.lock()
            _downloadGen += 1
            let gen = _downloadGen
            _downloadCont = continuation
            _downloadProgressAt = started
            lock.unlock()

            // Only this download may finish its continuation. Without
            // the check, this download's timeout (or a late reply)
            // could finish the next download with nil. Returns whether
            // this call finished it, so a check that runs after the
            // download ended is not logged as a timeout.
            func finish(
                _ value: Insta360DownloadResult?
            ) -> Bool {
                self.lock.lock()
                guard gen == self._downloadGen,
                      let cont = self._downloadCont
                else {
                    self.lock.unlock()
                    return false
                }
                self._downloadCont = nil
                self.lock.unlock()
                cont.resume(returning: value)
                return true
            }

            // The SDK sample downloads every file through the command
            // manager and never uses INSCameraHTTPManager.
            DispatchQueue.main.async {
                INSCameraManager.shared().commandManager.fetchResource(
                    withURI: uri,
                    toLocalFile: destURL,
                    progress: { _ in
                        let now = ProcessInfo.processInfo
                            .systemUptime
                        self.lock.lock()
                        if gen == self._downloadGen {
                            self._downloadProgressAt = now
                        }
                        self.lock.unlock()
                    }
                ) { error in
                    DispatchQueue.main.async {
                        if let error {
                            if finish(nil) {
                                self.onStatusUpdate?(
                                    "[\(cameraId)] download failed:"
                                    + " \(uri): \(error)"
                                )
                            }
                            return
                        }
                        // A file cut short (the camera still writing it,
                        // or a transfer that ended early) is deleted and
                        // counts as a failed download, so downloadPending
                        // queues it again. Only the current download
                        // deletes: a late reply from one given up on
                        // mustn't touch the next download's file.
                        guard Self.isCompletePhoto(destURL) else {
                            let bytes = (try? FileManager.default
                                .attributesOfItem(
                                    atPath: destURL.path
                                )[.size] as? Int) ?? 0
                            self.lock.lock()
                            let current = gen == self._downloadGen
                                && self._downloadCont != nil
                            self.lock.unlock()
                            if current {
                                try? FileManager.default
                                    .removeItem(at: destURL)
                            }
                            if finish(nil) {
                                self.onStatusUpdate?(
                                    "[\(cameraId)] download incomplete:"
                                    + " \(uri) (\(bytes) bytes, no INSP"
                                    + " trailer or JPEG end)"
                                )
                            }
                            return
                        }
                        let exifTs = Self
                            .extractExifTimestamp(
                                from: destURL
                            )
                        let done = finish(Insta360DownloadResult(
                            cameraId: cameraId,
                            scanIndex: scanIndex,
                            localURL: destURL,
                            mediaType: .equirectangular,
                            exifTimestamp: exifTs
                        ))
                        if done {
                            let bytes = (try? FileManager.default
                                .attributesOfItem(
                                    atPath: destURL.path
                                )[.size] as? Int) ?? 0
                            let detail = String(
                                format: "%.1f MB, %.1f s",
                                Double(bytes) / 1_048_576,
                                ProcessInfo.processInfo
                                    .systemUptime - started)
                            // X6 internal storage paths start with
                            // /storage_internal/, SD card paths with
                            // /DCIM/ (API doc, Get File List).
                            let from = uri.hasPrefix("/storage_internal/")
                                ? "internal" : "SD card"
                            self.onStatusUpdate?(
                                "[\(cameraId)] downloaded"
                                + " \(destURL.lastPathComponent)"
                                + " (\(detail)) from \(from): \(uri)"
                            )
                        }
                    }
                }
            }

            // Checks every 5 s; gives up after downloadStallSeconds
            // without progress or downloadMaxSeconds in total.
            func watchDownload() {
                DispatchQueue.main.asyncAfter(
                    deadline: .now() + 5
                ) {
                    let now = ProcessInfo.processInfo.systemUptime
                    self.lock.lock()
                    let pending = gen == self._downloadGen
                        && self._downloadCont != nil
                    let idle = now - self._downloadProgressAt
                    self.lock.unlock()
                    guard pending else { return }
                    let total = now - started
                    if idle < Self.downloadStallSeconds,
                       total < Self.downloadMaxSeconds
                    {
                        watchDownload()
                        return
                    }
                    if finish(nil) {
                        self.onStatusUpdate?(
                            "[\(cameraId)] download timeout"
                            + String(
                                format: " (%.0f s, %.0f s without data)",
                                total, idle)
                            + ": \(uri)"
                        )
                    }
                }
            }
            watchDownload()
        }
    }

    /// Whether a downloaded photo is complete: it ends with the Insta360
    /// trailer, as a complete INSP does, or, for a file without one,
    /// with the JPEG end-of-image marker (FF D9).
    private static func isCompletePhoto(_ url: URL) -> Bool {
        if InstaLensCalibration.hasTrailer(url: url) { return true }
        guard let fh = try? FileHandle(forReadingFrom: url) else {
            return false
        }
        defer { try? fh.close() }
        guard let size = try? fh.seekToEnd(), size >= 2,
              (try? fh.seek(toOffset: size - 2)) != nil,
              let end = try? fh.read(upToCount: 2)
        else { return false }
        return [UInt8](end) == [0xFF, 0xD9]
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
