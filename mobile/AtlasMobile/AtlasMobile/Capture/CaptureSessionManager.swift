import Foundation
import Combine
import ARKit
import CoreImage
import AudioToolbox
import UIKit

/// Orchestrates the full capture session: ARKit tracking, Insta360 cameras, and data recording.
@MainActor
final class CaptureSessionManager: ObservableObject {
    @Published var isSessionActive = false
    @Published var isReadyToCapture = false
    @Published var isAutoCapturing = false
    @Published var scanCount = 0
    @Published var connectedCameraCount = 0
    @Published var sessionDirectory: URL?
    @Published var showExportSheet = false
    @Published var exportError: String?
    @Published var cameraStatus: String?
    @Published var trackingState: ARCamera.TrackingState = .notAvailable
    @Published var showDepthOverlay = false
    @Published var showMesh = false
    @Published var showPointCloud = false
    @Published var depthOverlayImage: UIImage?
    @Published var calibCheckImage: UIImage?
    @Published var showCalibCheck = false
    @Published var isCheckingCalib = false
    @Published var isPipelineReady = false

    /// ARKit capture — exposed for CalibrationView.
    let arkitCapture = ARKitCapture()
    private var insta360Manager = Insta360CaptureManager()
    private var pipelineSink: AnyCancellable?
    private var camerasSink: AnyCancellable?
    private let maskManager = MaskManager()
    private var dataRecorder: DataRecorder?
    private var trajectoryRecorder: TrajectoryRecorder?

    /// Last downloaded Insta360 ERP — exposed for CalibrationView.
    @Published var lastInstaERP: UIImage?
    /// Thumbnail of the last captured iPhone RGB frame.
    @Published var lastCapturedThumbnail: UIImage?
    /// Triggers a brief flash animation on capture.
    @Published var captureFlash = false
    /// Monotonic counter — increment to force SwiftUI thumbnail refresh.
    @Published var thumbnailVersion: Int = 0
    /// Live connection diagnostic log for Insta360 cameras.
    @Published var connectionLog: [String] = []
    @Published var connectedCamerasList: [CameraInstance] = []
    @Published var disabledCameraIds: Set<String> = []

    @Published private(set) var isEndingSession = false
    @Published var teardownStage: String = ""
    @Published var captureTiming: String = ""
    private var connectTask: Task<Void, Never>?

    init() {
        arkitCapture.$trackingState
            .receive(on: DispatchQueue.main)
            .assign(to: &$trackingState)
        arkitCapture.$depthOverlayImage
            .receive(on: DispatchQueue.main)
            .assign(to: &$depthOverlayImage)
    }

    private func logMessage(_ msg: String) {
        connectionLog.append(msg)
        if connectionLog.count > 100 {
            connectionLog.removeFirst(
                connectionLog.count - 100
            )
        }
    }

    // MARK: - Per-camera toggles

    var enabledCameras: [CameraInstance] {
        connectedCamerasList.filter {
            !disabledCameraIds.contains($0.id)
        }
    }

    func isCameraEnabled(_ id: String) -> Bool {
        !disabledCameraIds.contains(id)
    }

    func toggleCamera(_ id: String) {
        if disabledCameraIds.contains(id) {
            disabledCameraIds.remove(id)
        } else {
            disabledCameraIds.insert(id)
        }
    }

    private func syncCamerasList() async {
        let cameras = await insta360Manager
            .connectedCamerasCopy()
        connectedCamerasList = cameras
    }

    func startSession() async {
        guard !isSessionActive, !isEndingSession else { return }

        showExportSheet = false
        exportError = nil
        lastCapturedThumbnail = nil
        captureTiming = "[\(Self.buildTag)]"
        teardownStage = ""
        isInsta360Busy = false
        insta360CommandCount = 0

        insta360Manager = Insta360CaptureManager()
        connectionLog = []
        connectedCamerasList = []
        disabledCameraIds = []
        insta360Manager.onLogMessage = { [weak self] msg in
            Task { @MainActor in
                self?.logMessage(msg)
            }
        }
        pipelineSink = insta360Manager.$isPipelineReady
            .receive(on: DispatchQueue.main)
            .sink { [weak self] ready in
                self?.isPipelineReady = ready
            }
        camerasSink = insta360Manager.$connectedCameras
            .receive(on: DispatchQueue.main)
            .sink { [weak self] cameras in
                self?.connectedCamerasList = cameras
                self?.connectedCameraCount = cameras.count
            }

        let sessionDir = SessionDirectory.create()
        dataRecorder = DataRecorder(sessionDirectory: sessionDir)
        sessionDirectory = sessionDir
        trajectoryRecorder = TrajectoryRecorder()

        arkitCapture.trajectoryRecorder = trajectoryRecorder
        arkitCapture.start()

        let configuredCount = insta360Manager.cameraConfig.cameras.count
        if configuredCount > 0 {
            cameraStatus = "Connecting \(configuredCount) camera\(configuredCount == 1 ? "" : "s")…"
        } else {
            cameraStatus = "iPhone only"
        }

        isSessionActive = true
        isReadyToCapture = true
        scanCount = 0

        setupDisconnectHandler()

        if configuredCount > 0 {
            let manager = insta360Manager
            let masks = maskManager
            connectTask = Task { [weak self] in
                await manager.discoverAndConnect()
                guard let self, self.isSessionActive else { return }
                let count = await manager.connectedCount()
                self.connectedCameraCount = count
                let cameras = await manager.connectedCamerasCopy()
                self.connectedCamerasList = cameras
                let syncInfo = await manager.clockOffsetSummary()
                let suffix = syncInfo.map { " (\($0))" } ?? ""
                if count == configuredCount {
                    self.cameraStatus = "\(count) camera\(count == 1 ? "" : "s") connected"
                        + suffix
                } else if count > 0 {
                    self.cameraStatus = "\(count)/\(configuredCount) cameras connected"
                        + suffix
                } else {
                    self.cameraStatus = "No cameras found"
                }
                masks.loadMasks(
                    for: cameras,
                    sessionDirectory: sessionDir
                )
            }
        }
    }


    func captureScan() async {
        guard isSessionActive, isReadyToCapture, !isEndingSession else { return }

        isReadyToCapture = false
        defer { isReadyToCapture = true }

        let t0 = ContinuousClock.now

        guard let arkitFrame = arkitCapture.captureCurrentFrame() else {
            return
        }

        let currentScan = scanCount
        scanCount += 1
        let fast = isAutoCapturing

        trajectoryRecorder?.recordPose(
            timestamp: arkitFrame.timestamp,
            pose: arkitFrame.pose
        )

        if !fast {
            captureFlash = true
        }
        let thumb = Self.makeThumbnail(
            from: arkitFrame.capturedImage
        )
        lastCapturedThumbnail = thumb
        thumbnailVersion += 1
        AudioServicesPlaySystemSound(1108)
        UIImpactFeedbackGenerator(style: .medium).impactOccurred()

        guard isSessionActive, !isEndingSession else { return }

        if fast {
            let activeCams = enabledCameras
            var sent360 = false
            if !isInsta360Busy, !activeCams.isEmpty,
               isAutoCapturing
            {
                isInsta360Busy = true
                insta360CommandCount += 1
                let cmdNum = insta360CommandCount
                sent360 = true
                let manager = insta360Manager
                let scan = currentScan
                let cams = activeCams
                insta360BackgroundTask?.cancel()
                insta360BackgroundTask = Task { [weak self] in
                    let captureStart = ContinuousClock.now
                    let arkitTs = CACurrentMediaTime()
                    let captureResults = await manager
                        .captureWith(
                            cameras: cams,
                            arkitTimestamp: arkitTs,
                            scanIndex: scan,
                            timeout: 30,
                            retryOnFailure: false
                        )
                    let capComp = (ContinuousClock.now
                        - captureStart).components
                    let captureMs = Int(capComp.seconds) * 1000
                        + Int(
                            capComp.attoseconds
                                / 1_000_000_000_000_000
                        )
                    self?.logMessage(
                        "360#\(cmdNum) scan#\(scan)"
                        + " ack=\(captureMs)ms"
                    )
                    self?.dataRecorder?
                        .updateInsta360Captures(
                            scanIndex: scan,
                            results: captureResults
                        )
                    self?.isInsta360Busy = false
                    guard let self,
                          let dir = self.sessionDirectory
                    else { return }
                    guard !Task.isCancelled else { return }
                    let downloads =
                        await manager.downloadAllPending(
                            into: dir
                        )
                    if !downloads.isEmpty {
                        let fname = downloads.first?
                            .localURL.lastPathComponent
                            ?? ""
                        self.logMessage(
                            "[360] dl=\(downloads.count)"
                            + " \(fname)"
                        )
                        self.dataRecorder?
                            .updateExposureTimestamps(
                                scanIndex: scan,
                                downloads: downloads
                            )
                        await self.updateInstaPreview(
                            from: downloads
                        )
                    }
                }
            }

            guard isAutoCapturing, isSessionActive,
                  !isEndingSession else { return }

            dataRecorder?.saveScan(
                scanIndex: currentScan,
                arkitFrame: arkitFrame,
                insta360Results: []
            )

            let t1 = ContinuousClock.now
            func ms(_ d: Duration) -> Int {
                let c = d.components
                return Int(c.seconds) * 1000
                    + Int(c.attoseconds / 1_000_000_000_000_000)
            }
            let i360tag = sent360
                ? " 360#\(insta360CommandCount)"
                : ""
            captureTiming = "[\(Self.buildTag)] #\(currentScan)"
                + " \(ms(t1 - t0))ms\(i360tag)"
        } else {
            let insta360Results: [Insta360CaptureResult]
            let manualCams = enabledCameras
            if !manualCams.isEmpty {
                insta360Results = await insta360Manager
                    .captureWith(
                        cameras: manualCams,
                        arkitTimestamp: arkitFrame.timestamp,
                        scanIndex: currentScan,
                        timeout: 30
                    )
            } else {
                insta360Results = []
            }

            dataRecorder?.saveScan(
                scanIndex: currentScan,
                arkitFrame: arkitFrame,
                insta360Results: insta360Results
            )

            let t1 = ContinuousClock.now
            func ms(_ d: Duration) -> Int {
                let c = d.components
                return Int(c.seconds) * 1000
                    + Int(c.attoseconds / 1_000_000_000_000_000)
            }
            captureTiming = "[\(Self.buildTag)] #\(currentScan)"
                + " \(ms(t1 - t0))ms"

            guard isSessionActive, !isEndingSession else { return }

            if !insta360Results.isEmpty, let dir = sessionDirectory {
                let downloads = await insta360Manager.downloadAllPending(
                    into: dir
                )
                await updateInstaPreview(from: downloads)
            }
        }
    }

    private func updateInstaPreview(
        from downloads: [Insta360DownloadResult]
    ) async {
        guard let first = downloads.first else {
            if !downloads.isEmpty {
                print("[Capture] Insta360 download:"
                    + " \(downloads.count) files")
            }
            return
        }
        let url = first.localURL
        let preview = await Task.detached(
            priority: .userInitiated
        ) { () -> UIImage? in
            guard let data = try? Data(contentsOf: url),
                  let img = UIImage(data: data)
            else { return nil }
            let maxW = 1024.0
            if img.size.width > maxW {
                let scale = maxW / img.size.width
                let sz = CGSize(
                    width: maxW,
                    height: img.size.height * scale
                )
                let renderer = UIGraphicsImageRenderer(
                    size: sz
                )
                return renderer.image { _ in
                    img.draw(
                        in: CGRect(origin: .zero, size: sz)
                    )
                }
            }
            return img
        }.value
        guard let preview else { return }
        self.lastInstaERP = preview
        self.thumbnailVersion += 1
    }

    private var autoCaptureTask: Task<Void, Never>?
    private var insta360BackgroundTask: Task<Void, Never>?

    private static let autoCaptureCooldown: UInt64 = 2_500_000_000
    private var isInsta360Busy = false
    private var insta360CommandCount = 0

    func startAutoCapture() {
        guard isSessionActive, !isAutoCapturing else { return }
        isAutoCapturing = true
        autoCaptureTask = Task {
            while isAutoCapturing && isSessionActive && !isEndingSession {
                let start = ContinuousClock.now
                await captureScan()
                let elapsed = ContinuousClock.now - start
                let minInterval = Duration.nanoseconds(
                    Self.autoCaptureCooldown
                )
                if elapsed < minInterval {
                    try? await Task.sleep(for: minInterval - elapsed)
                }
            }
            isAutoCapturing = false
        }
    }

    func stopAutoCapture() async {
        logMessage(
            "STOP: \(insta360CommandCount) cmds"
            + " / \(scanCount) scans"
        )
        isAutoCapturing = false

        let bgTask = insta360BackgroundTask
        insta360BackgroundTask = nil
        bgTask?.cancel()

        let task = autoCaptureTask
        autoCaptureTask = nil
        task?.cancel()
        _ = await task?.value
        _ = await bgTask?.value

        await insta360Manager.clearCaptureState()
        logMessage("STOP: task done")
    }

    private static let buildTag = "11e"

    func endSession() async {
        guard isSessionActive, !isEndingSession else { return }
        isEndingSession = true
        isReadyToCapture = false

        defer {
            teardownStage = "done"
            isEndingSession = false
            isSessionActive = false
            showExportSheet = true
        }

        teardownStage = "1-stopAuto"
        await stopAutoCapture()
        connectTask?.cancel()
        connectTask = nil
        insta360Manager.onCameraDisconnected = nil

        teardownStage = "2-stopARKit"
        arkitCapture.stop()

        let dir = sessionDirectory

        teardownStage = "3-downloading"
        if let dir {
            insta360Manager.saveClockOffsets(to: dir)
            let downloads =
                await insta360Manager.downloadAllPending(
                    into: dir
                )
            let byScan = Dictionary(
                grouping: downloads,
                by: \.scanIndex
            )
            for (scan, dls) in byScan {
                dataRecorder?.updateExposureTimestamps(
                    scanIndex: scan,
                    downloads: dls
                )
            }
            await dataRecorder?.saveDownloadedMedia(
                downloads
            )
        }

        teardownStage = "4-disconnect"
        await insta360Manager.disconnect()
        connectedCameraCount = 0
        connectedCamerasList = []

        teardownStage = "5-trajectory"
        await dataRecorder?.saveTrajectory(
            trajectoryRecorder?.export()
        )

        teardownStage = "6-export"
        lastInstaERP = nil
        lastCapturedThumbnail = nil
        depthOverlayImage = nil
        calibCheckImage = nil
        connectionLog = []
        connectedCamerasList = []
        trajectoryRecorder = nil

        let scans: [ScanExportData]
        let config = insta360Manager.cameraConfig
        if let dir, let recorder = dataRecorder {
            scans = recorder.buildExportData(
                sessionDirectory: dir
            )
        } else {
            scans = []
        }
        dataRecorder = nil

        guard dir != nil, !scans.isEmpty else { return }
    }

    // MARK: - Visualization toggles

    func toggleDepthOverlay() {
        showDepthOverlay.toggle()
        arkitCapture.showDepthOverlay = showDepthOverlay
    }

    func toggleMesh() {
        showMesh.toggle()
        arkitCapture.showMesh = showMesh
    }

    func togglePointCloud() {
        showPointCloud.toggle()
    }

    func dismissCalibCheck() {
        showCalibCheck = false
        calibCheckImage = nil
    }

    func quickCalibrationCheck() async {
        guard !isCheckingCalib else { return }
        isCheckingCalib = true
        defer { isCheckingCalib = false }

        AudioServicesPlaySystemSound(1108)
        // Snapshot the ARKit frame at shutter time so
        // the depth overlay matches the 360° capture,
        // not the user's position seconds later after
        // download + stitch complete.
        let shutterFrame = arkitCapture.session?
            .currentFrame
        let captured = await triggerInsta360Capture()
        var erpImage: UIImage?
        if captured,
           let raw = await downloadLastInsta360Image()
        {
            if DualFisheyeStitcher.isDualFisheye(raw) {
                let src = raw
                let camModel = await insta360Manager
                    .connectedCamerasCopy().first?.model ?? ""
                let roll = DualFisheyeStitcher
                    .rollCorrection(forModel: camModel)
                erpImage = await Task.detached(
                    priority: .userInitiated
                ) {
                    DualFisheyeStitcher.stitch(
                        src, outputWidth: 2048,
                        rollDeg: roll
                    )
                }.value
            } else {
                erpImage = raw
            }
        }

        guard let erp = erpImage else {
            print(
                "[CalibCheck] no ERP image available"
            )
            return
        }

        guard let frame = shutterFrame else { return }

        let depthMap = frame.sceneDepth?.depthMap
            ?? frame.smoothedSceneDepth?.depthMap
        guard let dm = depthMap else { return }
        CVPixelBufferLockBaseAddress(dm, .readOnly)
        defer {
            CVPixelBufferUnlockBaseAddress(
                dm, .readOnly
            )
        }
        let depthW = CVPixelBufferGetWidth(dm)
        let depthH = CVPixelBufferGetHeight(dm)
        guard let base = CVPixelBufferGetBaseAddress(
            dm
        ) else { return }
        let floats = base.assumingMemoryBound(
            to: Float32.self
        )
        let depthArr: [Float] = Array(
            UnsafeBufferPointer(
                start: floats,
                count: depthW * depthH
            )
        )

        let bufW = CVPixelBufferGetWidth(
            frame.capturedImage
        )
        let bufH = CVPixelBufferGetHeight(
            frame.capturedImage
        )

        let config = MultiCameraConfig
            .loadFromDeviceOrBundle()
        let connectedId = await insta360Manager
            .connectedCamerasCopy().first?.id
        guard let camCfg = config.cameras
            .first(where: {
                $0.id == connectedId
            }) ?? config.cameras
            .first(where: { $0.enabled })
        else { return }

        let resW = Int(
            frame.camera.imageResolution.width
        )
        let resH = Int(
            frame.camera.imageResolution.height
        )

        let calibFrame = CalibrationFrame(
            depth: depthArr,
            depthW: depthW, depthH: depthH,
            intrinsics: frame.camera.intrinsics,
            imageW: bufW, imageH: bufH,
            imageResW: resW, imageResH: resH,
            erpImage: erp,
            iphoneImage: nil
        )

        let overlay = CalibrationOverlayRenderer
            .render(
                frame: calibFrame,
                extrinsic: camCfg.extrinsic
            )

        calibCheckImage = overlay?.depthOnErp
            ?? overlay?.erpSlice
        showCalibCheck = overlay != nil
    }

    // MARK: - Calibration capture

    func prepareInsta360ForCapture() async -> Bool {
        if let task = insta360BackgroundTask {
            task.cancel()
            _ = await task.value
            insta360BackgroundTask = nil
        }

        if let task = connectTask {
            _ = await task.value
        }

        var count = await insta360Manager.connectedCount()
        await syncCamerasList()
        logMessage(
            "[Calib] prepare: count=\(count)"
        )

        if count == 0 {
            logMessage(
                "[Calib] no cameras — reconnecting"
            )
            await insta360Manager.discoverAndConnect()
            count = await insta360Manager
                .connectedCount()
            await syncCamerasList()
        }
        guard count > 0 else { return false }

        await insta360Manager.prepareCameras()
        logMessage("[Calib] cameras ready")
        return true
    }

    func fireInsta360Capture() async -> Bool {
        let results = await insta360Manager.captureAll(
            arkitTimestamp: CACurrentMediaTime(),
            scanIndex: -1,
            timeout: 150,
            skipPreChecks: true
        )
        logMessage(
            "[Calib] results=\(results.count)"
            + " ids=\(results.map(\.mediaIdentifier))"
        )
        return results.contains(where: {
            !$0.mediaIdentifier.isEmpty
        })
    }

    func triggerInsta360Capture() async -> Bool {
        let ready = await prepareInsta360ForCapture()
        guard ready else { return false }
        return await fireInsta360Capture()
    }

    func downloadLastInsta360Image() async -> UIImage? {
        guard let dir = sessionDirectory else {
            print("[Insta360] download skipped — no session directory")
            return nil
        }
        let downloads = await insta360Manager.downloadAllPending(
            into: dir
        )
        guard let first = downloads.first,
              let data = try? Data(contentsOf: first.localURL),
              let img = UIImage(data: data)
        else {
            print("[Insta360] download failed or timed out")
            return nil
        }
        let maxW = 1024.0
        if img.size.width > maxW {
            let scale = maxW / img.size.width
            let sz = CGSize(
                width: maxW,
                height: img.size.height * scale
            )
            let renderer = UIGraphicsImageRenderer(
                size: sz
            )
            let thumb = renderer.image { _ in
                img.draw(
                    in: CGRect(origin: .zero, size: sz)
                )
            }
            lastInstaERP = thumb
            return thumb
        }
        lastInstaERP = img
        return img
    }

    // MARK: - Insta360 retry

    func retryInsta360Connection() {
        let configuredCount = insta360Manager.cameraConfig.cameras.count
        guard configuredCount > 0 else { return }
        cameraStatus = "Retrying \(configuredCount) camera\(configuredCount == 1 ? "" : "s")…"
        logMessage("--- Retry started ---")

        let manager = insta360Manager
        Task { [weak self] in
            await manager.retryConnection()
            guard let self else { return }
            let count = await manager.connectedCount()
            let total = configuredCount
            self.connectedCameraCount = count
            await self.syncCamerasList()
            let syncInfo = await manager.clockOffsetSummary()
            let suffix = syncInfo.map { " (\($0))" } ?? ""
            if count == total {
                self.cameraStatus = "\(count) camera\(count == 1 ? "" : "s") connected"
                    + suffix
            } else if count > 0 {
                self.cameraStatus = "\(count)/\(total) cameras connected"
                    + suffix
            } else {
                self.cameraStatus = "No cameras found"
            }
        }
    }

    private func setupDisconnectHandler() {
        insta360Manager.onCameraDisconnected = { [weak self] cameraId in
            Task { @MainActor in
                guard let self, self.isSessionActive else { return }
                let count = await self.insta360Manager.connectedCount()
                let total = self.insta360Manager.cameraConfig.cameras.count
                self.connectedCameraCount = count
                await self.syncCamerasList()
                if count == 0 {
                    self.cameraStatus = "Camera disconnected"
                } else {
                    self.cameraStatus = "\(count)/\(total) cameras"
                }
            }
        }
    }

    private nonisolated static let thumbContext = CIContext(
        options: [.useSoftwareRenderer: true]
    )

    nonisolated static func makeThumbnail(
        from buffer: CVPixelBuffer
    ) -> UIImage? {
        autoreleasepool {
            let ciImage = CIImage(cvPixelBuffer: buffer)
            guard ciImage.extent.width > 0 else {
                return nil
            }
            let scale = 160.0 / ciImage.extent.width
            let scaled = ciImage.transformed(
                by: CGAffineTransform(scaleX: scale, y: scale)
            )
            guard let cgImage = thumbContext.createCGImage(
                scaled, from: scaled.extent
            ) else {
                return nil
            }
            return UIImage(
                cgImage: cgImage, scale: 1.0, orientation: .right
            )
        }
    }
}
