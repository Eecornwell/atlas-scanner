import Foundation
import UIKit
import INSCameraSDK
import INSCoreMedia

/// The live preview of the Insta360 SDK sample's take-picture page,
/// ported from INSCameraSDKSample-bluetooth/src/
/// CameraConfigByJsonController.swift (the page "Record new" opens). On
/// that page the preview runs while "X5 Take Picture" is pressed, and the
/// X6 takes photos there, so CameraInstance waits for it before a shot.
///
/// The page's steps:
/// - viewDidLoad (setupRenderView): create an INSCameraSessionPlayer.
/// - viewWillAppear: fetchOptions, then disableInternalSplicingForPreview,
///   then runMediaSession, which starts the player and requests an
///   I-frame.
/// - cameraState KVO: .connected runs runMediaSession; a lost connection
///   stops the stream.
/// - INSCameraCurrentCaptureStatus: at .notCapture (the camera is idle
///   again after a shot), fetchOptions, then the media session's
///   stopRunning, then runMediaSession restarts the stream.
///
/// State is used on the main queue, where the sample makes its calls.
final class Insta360LivePreview: NSObject, ObservableObject {
    static let shared = Insta360LivePreview()

    /// True while the player's stream runs.
    @Published private(set) var isRunning = false
    /// True from start() until stop().
    @Published private(set) var isActive = false

    /// Receives status lines. Set and called on the main queue.
    var onLogMessage: ((String) -> Void)?

    /// The player's render view, shown by Insta360PreviewTile. Nil until
    /// the first start().
    var renderView: UIView? { player?.renderView }

    private var player: INSCameraSessionPlayer?
    /// The sample keeps a media session next to the player: it sets the
    /// decoder config on it, and stops it when the connection drops and
    /// before each restart.
    private lazy var mediaSession = INSCameraMediaSession()

    /// Read by fetchOptions. The X6 can stream H.265, and decoding it as
    /// the default H.264 gives a black preview (sample comment).
    private var videoEncode: INSVideoEncode = .H264
    private var windowCropInfo: INSWindowCropInfo?
    private var rollingShutterTimeMs: Double = 0

    /// A player start is in progress.
    private var isStarting = false
    /// Numbers the starts, so a late reply from a timed-out start is
    /// ignored.
    private var startAttempt = 0
    /// A restart after a shot is queued but hasn't reached
    /// runMediaSession yet. The sample's page blocks its buttons while
    /// the preview restarts, so shots wait for this too.
    private var isRestartPending = false
    private var startWaiters: [(Bool) -> Void] = []
    private var stateToken: NSKeyValueObservation?
    private var captureStatusObserver: NSObjectProtocol?
    private var lastCaptureStatus = ""
    private var lastPlayerError = ""

    /// Cameras whose offset the sample converts in getMediaOffset. The
    /// X6 (C9) isn't one of them.
    private static let convertedOffsetCameras: Set<String> = [
        kInsta360CameraNameOne,
        kInsta360CameraNameOneR,
        kInsta360CameraNameOneX2,
        kInsta360CameraNameX3,
        kInsta360CameraNameX4,
        kInsta360CameraNameOneH,
        kInsta360CameraNameOneRS,
    ]

    private override init() {
        super.init()
    }

    // MARK: - Control

    /// Starts the preview the way the sample's page does when it opens.
    /// Returns whether the stream runs. Does nothing if it already runs.
    func start() async -> Bool {
        await awaitOnMain(timeout: 30, fallback: false) { finish in
            self.makePlayerIfNeeded()
            self.isActive = true
            self.observeCamera()
            if self.isRunning {
                finish(true)
                return
            }
            self.fetchOptions {
                self.disableInternalSplicing {
                    self.runMediaSession(completion: finish)
                }
            }
        }
    }

    /// Stops the stream, as the sample's page does when it closes.
    func stop() async {
        _ = await awaitOnMain(timeout: 5, fallback: false) { finish in
            guard self.isActive else {
                finish(true)
                return
            }
            self.isActive = false
            self.isRestartPending = false
            self.stateToken = nil
            if let token = self.captureStatusObserver {
                NotificationCenter.default.removeObserver(token)
                self.captureStatusObserver = nil
            }
            self.isRunning = false
            self.mediaSession.stopRunning { _ in }
            guard let player = self.player else {
                finish(true)
                return
            }
            player.stopRunning { _ in
                DispatchQueue.main.async {
                    self.log("live preview stopped")
                    finish(true)
                }
            }
        }
    }

    /// Waits until the stream runs: in the sample, the preview is on when
    /// the take-picture button is pressed. If the stream is neither
    /// running nor starting, starts it once. Returns false when that start
    /// fails, at the timeout, or at once before start().
    func waitUntilRunning(timeout: Double) async -> Bool {
        let deadline = ProcessInfo.processInfo.systemUptime + timeout
        var restarted = false
        while true {
            let (active, running, busy) = await MainActor.run {
                (
                    self.isActive,
                    self.isRunning,
                    self.isStarting || self.isRestartPending
                )
            }
            if running { return true }
            if !active { return false }
            if !busy {
                if restarted { return false }
                restarted = true
                await MainActor.run {
                    self.log("live preview not running — starting it")
                    self.runMediaSession { _ in }
                }
                continue
            }
            if ProcessInfo.processInfo.systemUptime >= deadline {
                return false
            }
            try? await Task.sleep(for: .milliseconds(250))
        }
    }

    // MARK: - The sample page's steps

    /// setupRenderView.
    private func makePlayerIfNeeded() {
        guard player == nil else { return }
        let newPlayer = INSCameraSessionPlayer()
        newPlayer.delegate = self
        newPlayer.dataSource = self
        newPlayer.needCameraPreviewStreamAutoRotate = true
        newPlayer.render.renderModelType.displayType = .sphereStitch
        // The sample sizes the view before adding it; this is the tile's
        // size until the tile lays it out.
        newPlayer.renderView.frame = CGRect(
            x: 0, y: 0, width: 150, height: 100
        )

        let config = INSH264DecoderConfig()
        config.shouldReloadDecoder = false
        config.decodeErrorMaxCount = 30
        config.debugLiveStrem = false
        config.debugLog = false
        mediaSession.setDecoderConfig(config)

        player = newPlayer
    }

    /// fetchOptions: reads the stream's encoding and crop info for the
    /// player, and the rolling shutter time for its stabilizer.
    private func fetchOptions(_ completion: @escaping () -> Void) {
        let optionTypes: [NSNumber] = [
            NSNumber(value: INSCameraOptionsType.storageState.rawValue),
            NSNumber(value: INSCameraOptionsType.evoStatus.rawValue),
            NSNumber(value: INSCameraOptionsType.videoEncode.rawValue),
            NSNumber(value: INSCameraOptionsType.videoSubMode.rawValue),
            NSNumber(value: INSCameraOptionsType.windowCropInfo.rawValue),
            NSNumber(value: INSCameraOptionsType.photoSubMode.rawValue),
            NSNumber(
                value: INSCameraOptionsType.rollingShutterTime.rawValue
            ),
            NSNumber(value: INSCameraOptionsType.gyroTimestamp.rawValue),
        ]
        INSCameraManager.shared().commandManager.getOptionsWithTypes(
            optionTypes
        ) { [weak self] error, options, _ in
            DispatchQueue.main.async {
                guard let self else {
                    completion()
                    return
                }
                if let options {
                    self.windowCropInfo = options.windowCropInfo
                    self.rollingShutterTimeMs = options.rollingShutterTime
                    self.videoEncode = options.videoEncode
                    self.log(
                        "camera options: photo sub mode"
                        + " \(options.photoSubMode), video sub mode"
                        + " \(options.videoSubMode), stream encode"
                        + " \(options.videoEncode.rawValue)"
                    )
                } else {
                    self.log(
                        "camera options read failed: "
                        + (error.map { "\($0)" } ?? "no options")
                    )
                }
                completion()
            }
        }
    }

    /// disableInternalSplicingForPreview: the preview stitches the two
    /// fisheye images in the app, so the sample turns in-camera
    /// stitching off first. Its comment: with in-camera stitching on,
    /// the seams look wrong and parts of the image repeat.
    private func disableInternalSplicing(
        _ completion: @escaping () -> Void
    ) {
        let cameraOptions = INSCameraOptions()
        cameraOptions.enableInternalSplicing = false
        let optionTypes = [
            NSNumber(value: INSCameraOptionsType.internalSplicing.rawValue),
        ]
        INSCameraManager.shared().commandManager.setOptions(
            cameraOptions, forTypes: optionTypes
        ) { [weak self] error, _ in
            DispatchQueue.main.async {
                if let error {
                    self?.log("in-camera stitching off failed: \(error)")
                }
                completion()
            }
        }
    }

    /// runMediaSession: starts the player, or stops and starts it if it
    /// runs, then requests an I-frame. `completion` gets whether the
    /// stream runs. A call during a start gets that start's result.
    private func runMediaSession(completion: @escaping (Bool) -> Void) {
        guard isActive, let player,
              INSCameraManager.shared().cameraState == .connected
        else {
            completion(false)
            return
        }
        startWaiters.append(completion)
        guard !isStarting else { return }
        isStarting = true
        isRunning = false
        startAttempt += 1
        let attempt = startAttempt
        mediaSession.videoStreamEncode = videoEncode
        player.videoStreamEncode = videoEncode
        // The sample sets this for the ONE RS only.
        if INSCameraManager.shared().currentCamera?.name
            == kInsta360CameraNameOneRS
        {
            player.expectedVideoResolution = INSVideoResolution1440x720x30
        }
        let startPlayer = { [weak self] in
            player.startRunning { error in
                DispatchQueue.main.async {
                    self?.didStart(attempt: attempt, error: error)
                }
            }
        }
        if player.running {
            player.stopRunning { _ in
                DispatchQueue.main.async {
                    startPlayer()
                }
            }
        } else {
            startPlayer()
        }
        // An SDK completion can fail to fire, for example when the
        // connection drops during the start.
        DispatchQueue.main.asyncAfter(deadline: .now() + 20) {
            [weak self] in
            self?.didStart(
                attempt: attempt,
                error: NSError(
                    domain: "Insta360LivePreview", code: 1,
                    userInfo: [
                        NSLocalizedDescriptionKey:
                            "no reply to startRunning in 20 s",
                    ]
                )
            )
        }
    }

    /// Ends start attempt `attempt`. Ignored if that attempt has already
    /// ended.
    private func didStart(attempt: Int, error: Error?) {
        guard isStarting, attempt == startAttempt else { return }
        isStarting = false
        let waiters = startWaiters
        startWaiters = []
        var ok = false
        if !isActive {
            // stop() ran while the stream was starting.
            player?.stopRunning { _ in }
        } else if let error {
            log("live preview start failed: \(error)")
        } else {
            ok = true
            isRunning = true
            lastPlayerError = ""
            log("live preview running")
            INSCameraManager.shared().commandManager.requestIFrame { _ in }
        }
        waiters.forEach { $0(ok) }
    }

    // MARK: - Camera events

    private func observeCamera() {
        if stateToken == nil {
            stateToken = INSCameraManager.socket().observe(
                \.cameraState, options: [.new]
            ) { [weak self] manager, _ in
                let state = manager.cameraState
                DispatchQueue.main.async {
                    self?.cameraStateChanged(state)
                }
            }
        }
        if captureStatusObserver == nil {
            captureStatusObserver = NotificationCenter.default.addObserver(
                forName: .INSCameraCurrentCaptureStatus,
                object: nil,
                queue: .main
            ) { [weak self] note in
                guard let status = note.userInfo?["status"]
                    as? INSCameraCaptureStatus
                else { return }
                self?.captureStatusChanged(status)
            }
        }
    }

    /// The sample's cameraState KVO. .found and .synchronized change
    /// nothing.
    private func cameraStateChanged(_ state: INSCameraState) {
        guard isActive else { return }
        switch state {
        case .connected:
            log("camera connected — starting live preview")
            runMediaSession { _ in }
        case .connectFailed, .noConnection:
            isRunning = false
            mediaSession.stopRunning { _ in }
            player?.stopRunning { _ in }
        default:
            break
        }
    }

    /// The sample's INSCameraCurrentCaptureStatus handler. It restarts
    /// the preview only at .notCapture; its comment says a restart while
    /// the camera is still saving fails with "start media failed".
    private func captureStatusChanged(_ status: INSCameraCaptureStatus) {
        let text = "state \(status.state.rawValue),"
            + " submode \(status.captureSubmode.rawValue)"
        if text != lastCaptureStatus {
            lastCaptureStatus = text
            log("camera capture status: \(text)")
        }
        guard isActive, status.state == .notCapture, !isRestartPending
        else { return }
        isRestartPending = true
        // Restarts once: in the media session's stopRunning completion,
        // as the sample does, or after 10 s if a completion never fires.
        var restarted = false
        let restart = { [weak self] in
            guard let self, !restarted else { return }
            restarted = true
            self.isRestartPending = false
            self.runMediaSession { _ in }
        }
        fetchOptions { [weak self] in
            guard let self else { return }
            self.mediaSession.stopRunning { _ in
                DispatchQueue.main.async { restart() }
            }
        }
        DispatchQueue.main.asyncAfter(deadline: .now() + 10) {
            restart()
        }
    }

    // MARK: - Helpers

    /// getMediaOffset of the sample's page. For the X6 this is the
    /// camera's mediaOffsetV6, else its mediaOffset.
    private func mediaOffset() -> String? {
        let camera = INSCameraManager.shared().currentCamera
        let settings: INSCameraDeviceSettings? = camera?.settings
        var mediaOffset = settings?.mediaOffsetV6 ?? settings?.mediaOffset
        guard let name = camera?.name,
              Self.convertedOffsetCameras.contains(name),
              let offset = mediaOffset,
              INSLensOffset.isValidOffset(offset)
        else { return mediaOffset }

        var converted = offset
        let cropValid = windowCropInfo.map {
            $0.srcWidth > 0 && $0.srcHeight > 0
                && $0.dstWidth > 0 && $0.dstHeight > 0
        } ?? false
        // The sample skips the crop for an X4 that reported invalid crop
        // info. The player can ask for the offset often, so this goes to
        // the console only.
        if name == kInsta360CameraNameX4 && !cropValid {
            print("[Insta360] X4 crop info invalid, offset not cropped")
        } else if let crop = windowCropInfo {
            converted = INSOffsetCalculator.cropOffset(
                converted, srcWidth: Int32(crop.srcWidth),
                srcHeight: Int32(crop.srcHeight),
                dstWidth: Int32(crop.dstWidth),
                dstHeight: Int32(crop.dstHeight),
                xOffset: crop.cropOffsetX,
                yOffset: crop.cropOffsetY)
        }
        mediaOffset = converted
        // The sample skips this conversion for the X4 and ONE RS.
        if name != kInsta360CameraNameX4
            && name != kInsta360CameraNameOneRS
        {
            mediaOffset = INSOffsetCalculator.convertOffset(
                converted, to: .oneX3040_2_2880)
        }
        return mediaOffset
    }

    /// Logs on the main queue.
    private func log(_ message: String) {
        let send = {
            print("[Insta360] \(message)")
            self.onLogMessage?(message)
        }
        if Thread.isMainThread {
            send()
        } else {
            DispatchQueue.main.async { send() }
        }
    }
}

// MARK: - INSCameraSessionPlayer data source and delegate

// The data source methods are the sample page's, unchanged.
extension Insta360LivePreview: INSCameraSessionPlayerDataSource,
    INSCameraSessionPlayerDelegate
{
    func updateOffset(to player: INSCameraSessionPlayer) -> String? {
        mediaOffset()
    }

    func updateRenderModelType(
        to player: INSCameraSessionPlayer,
        renderModelType: INSRenderModelType
    ) -> INSRenderModelType {
        renderModelType.displayType = .sphereStitch
        renderModelType.imageLayout = .horizontalMerged
        renderModelType.isHalfFisheye = false
        renderModelType.isSelfieVideo = false
        renderModelType.touchMode = false
        renderModelType.opticalFlowType = .disflow
        renderModelType.isHalfFisheyeBulletTime = false
        renderModelType.contentMode = .fitScreen
        renderModelType.preferDynamicVertex = false
        renderModelType.aiFlowBottomPercision = .unknown
        renderModelType.dynamicAlphaFlag = false
        renderModelType.usingFisheyeMask = false

        if let windowCropInfo {
            renderModelType.cropInfo = INSCropInfo()
            renderModelType.cropInfo.srcWidth = Int32(windowCropInfo.srcWidth)
            renderModelType.cropInfo.srcHeight = Int32(windowCropInfo.srcHeight)
            renderModelType.cropInfo.dstWidth = Int32(windowCropInfo.dstWidth)
            renderModelType.cropInfo.dstHeight = Int32(windowCropInfo.dstHeight)
        }

        renderModelType.aiFlowVersion = 1
        renderModelType.expandFlowWorkRegion = true
        renderModelType.aiFlowFrameInterval = 2
        renderModelType.colorFusion = true
        renderModelType.dynamicStitchType = .dynamicVideo
        renderModelType.cameraType =
            INSCameraManager.shared().currentCamera?.cameraType ?? ""
        return renderModelType
    }

    func updateStabilizerParam(
        to player: INSCameraSessionPlayer
    ) -> INSRealtimeStabilizerParam {
        let param = INSRealtimeStabilizerParam()
        if let offset = mediaOffset() {
            param.offset = offset
        }
        param.preferredStabMode = .still
        param.maxFilterAngleDegree = 25
        param.sweepTime = rollingShutterTimeMs
        param.windSize = 3
        param.fps = 30.0
        return param
    }

    func updateStabilizerDynamicParam(
        to player: INSCameraSessionPlayer,
        dynamicParam: INSStabilizerDynamicParam
    ) -> INSStabilizerDynamicParam {
        let param = dynamicParam
        param.onlineFilterType = .pathPlanSlidingWin
        return param
    }

    /// The SDK calls this when the player shows its first frame.
    func playerDidSetup(_ player: INSCameraSessionPlayer) {
        log("live preview: first frame shown")
    }

    func player(
        _ player: INSCameraSessionPlayer,
        didOccurWithError error: Error
    ) {
        let text = "live preview error: \(error)"
        DispatchQueue.main.async {
            // A failing stream reports the same error over and over.
            guard text != self.lastPlayerError else { return }
            self.lastPlayerError = text
            self.log(text)
        }
    }
}

/// Runs `body` on the main queue, where the SDK sample makes its calls,
/// and returns the value `body` passes to `finish`. Only the first value
/// counts. Returns `fallback` if `finish` isn't called within `timeout`
/// seconds: an SDK completion block can fail to fire, for example when
/// the connection drops.
private func awaitOnMain<T>(
    timeout: Double,
    fallback: T,
    _ body: @escaping (_ finish: @escaping (T) -> Void) -> Void
) async -> T {
    await withCheckedContinuation {
        (continuation: CheckedContinuation<T, Never>) in
        let lock = NSLock()
        var finished = false

        func finish(_ value: T) {
            lock.lock()
            let first = !finished
            finished = true
            lock.unlock()
            if first {
                continuation.resume(returning: value)
            }
        }

        DispatchQueue.main.async {
            body(finish)
        }
        DispatchQueue.main.asyncAfter(deadline: .now() + timeout) {
            finish(fallback)
        }
    }
}
