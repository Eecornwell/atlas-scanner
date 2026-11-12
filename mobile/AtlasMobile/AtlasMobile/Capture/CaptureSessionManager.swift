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
    @Published var speedMps: Double = 0
    @Published var angularRateDps: Double = 0
    @Published var smoothedAngularRateDps: Double = 0
    @Published var freeSpaceGB: Double = 0
    @Published var x5HoldSteady = false
    private var x5HoldTask: Task<Void, Never>?
    private var connectTask: Task<Void, Never>?
    private var diskSpaceTimer: Timer?

    init() {
        arkitCapture.$trackingState
            .receive(on: DispatchQueue.main)
            .assign(to: &$trackingState)
        arkitCapture.$depthOverlayImage
            .receive(on: DispatchQueue.main)
            .assign(to: &$depthOverlayImage)
        arkitCapture.$speedMps
            .receive(on: DispatchQueue.main)
            .assign(to: &$speedMps)
        arkitCapture.$angularRateDps
            .receive(on: DispatchQueue.main)
            .assign(to: &$angularRateDps)
        arkitCapture.$smoothedAngularRateDps
            .receive(on: DispatchQueue.main)
            .assign(to: &$smoothedAngularRateDps)
        updateFreeSpace()
    }

    private func startX5HoldWindow() {
        x5HoldSteady = true
        x5HoldTask?.cancel()
        x5HoldTask = Task { @MainActor in
            try? await Task.sleep(for: .seconds(1.5))
            if !Task.isCancelled {
                x5HoldSteady = false
            }
        }
    }

    private func recordMotionWindow(
        scanIndex: Int,
        startTime: TimeInterval
    ) async -> (peakDps: Double, meanDps: Double) {
        var peak: Double = 0
        var samples: [Double] = []
        let endTime = startTime + 1.5
        while ProcessInfo.processInfo.systemUptime < endTime {
            let rate = angularRateDps
            peak = max(peak, rate)
            samples.append(rate)
            try? await Task.sleep(for: .milliseconds(50))
        }
        let mean = samples.isEmpty
            ? 0
            : samples.reduce(0, +) / Double(samples.count)
        return (peakDps: peak, meanDps: mean)
    }

    private func logMessage(_ msg: String) {
        connectionLog.append(msg)
        if connectionLog.count > 100 {
            connectionLog.removeFirst(
                connectionLog.count - 100
            )
        }
        appendToCaptureLog(msg)
    }

    /// Keeps every status line in the session folder (capture_log.txt):
    /// the on-screen log holds only the last 100 lines and is cleared at
    /// the next session. Lines start with systemUptime, the clock of the
    /// command timestamps in pose.json.
    private func appendToCaptureLog(_ msg: String) {
        guard let dir = sessionDirectory else { return }
        let stamp = String(
            format: "%10.3f ",
            ProcessInfo.processInfo.systemUptime)
        guard let data = (stamp + msg + "\n").data(using: .utf8)
        else { return }
        let url = dir.appendingPathComponent("capture_log.txt")
        if let handle = try? FileHandle(forWritingTo: url) {
            defer { try? handle.close() }
            _ = try? handle.seekToEnd()
            try? handle.write(contentsOf: data)
        } else {
            try? data.write(to: url)
        }
    }

    private func updateFreeSpace() {
        let home = URL(fileURLWithPath: NSHomeDirectory())
        if let vals = try? home.resourceValues(
            forKeys: [.volumeAvailableCapacityForImportantUsageKey]
        ), let bytes = vals.volumeAvailableCapacityForImportantUsage {
            freeSpaceGB = Double(bytes) / 1_073_741_824
        }
    }

    private func startDiskSpaceMonitor() {
        updateFreeSpace()
        diskSpaceTimer = Timer.scheduledTimer(
            withTimeInterval: 5, repeats: true
        ) { [weak self] _ in
            DispatchQueue.main.async {
                self?.updateFreeSpace()
            }
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
        startDiskSpaceMonitor()

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
        scanInProgress = true
        defer {
            isReadyToCapture = true
            scanInProgress = false
        }

        let t0 = ContinuousClock.now

        guard let arkitFrame = arkitCapture.captureCurrentFrame() else {
            return
        }
        let frameAge = CACurrentMediaTime()
            - arkitFrame.timestamp
        let nearestTrajTs = trajectoryRecorder?
            .nearestTimestamp(to: arkitFrame.timestamp)

        let currentScan = scanCount
        scanCount += 1
        let fast = isAutoCapturing

        let frameAgeMs = frameAge * 1000
        let trajDeltaMs: Double? = nearestTrajTs.map {
            (arkitFrame.timestamp - $0) * 1000
        }
        let trajDelta: String
        if let td = trajDeltaMs {
            trajDelta = String(
                format: "trajΔ=%.1fms", td
            )
        } else {
            trajDelta = "trajΔ=n/a"
        }
        logMessage(
            String(
                format: "scan#%d age=%.1fms %@",
                currentScan,
                frameAgeMs,
                trajDelta
            )
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
            let motionOk = smoothedAngularRateDps < 2.0
            if !isInsta360Busy, !activeCams.isEmpty,
               isAutoCapturing, motionOk
            {
                isInsta360Busy = true
                insta360CommandCount += 1
                let cmdNum = insta360CommandCount
                sent360 = true
                startX5HoldWindow()
                let manager = insta360Manager
                let scan = currentScan
                let cams = activeCams
                insta360BackgroundTask?.cancel()
                insta360BackgroundTask = Task { [weak self] in
                    // Busy until this shot has also downloaded, so the
                    // next shot cannot start a second download on the
                    // same camera while this one runs (they share one
                    // download continuation in CameraInstance).
                    defer { self?.isInsta360Busy = false }
                    let cmdTime = ProcessInfo.processInfo
                        .systemUptime
                    let captureStart = ContinuousClock.now
                    let arkitTs = CACurrentMediaTime()
                    let motionTask = Task {
                        @MainActor [weak self] in
                        await self?.recordMotionWindow(
                            scanIndex: scan,
                            startTime: cmdTime
                        )
                    }
                    // 60 s, not 30: a reply that arrives after the
                    // timeout is dropped and its photo never downloaded.
                    // The longer wait leaves room for 120 MP X6 photos.
                    let captureResults = await manager
                        .captureWith(
                            cameras: cams,
                            arkitTimestamp: arkitTs,
                            scanIndex: scan,
                            timeout: 60
                        )
                    let capComp = (ContinuousClock.now
                        - captureStart).components
                    let captureMs = Int(capComp.seconds) * 1000
                        + Int(
                            capComp.attoseconds
                                / 1_000_000_000_000_000
                        )
                    let motion = await motionTask.value
                    self?.logMessage(String(
                        format: "360#%d scan#%d"
                            + " ack=%dms"
                            + " peak=%.1f°/s"
                            + " mean=%.1f°/s",
                        cmdNum, scan, captureMs,
                        motion?.peakDps ?? 0,
                        motion?.meanDps ?? 0
                    ))
                    self?.dataRecorder?
                        .updateInsta360Captures(
                            scanIndex: scan,
                            results: captureResults,
                            peakAngularRateDps:
                                motion?.peakDps,
                            meanAngularRateDps:
                                motion?.meanDps
                        )
                    guard let self,
                          let dir = self.sessionDirectory
                    else { return }
                    // No cancellation check here: once the shutter has
                    // fired, the photo should be downloaded.
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
                        // A retried download can belong to an earlier
                        // scan, so record each under its own scan.
                        let byScan = Dictionary(
                            grouping: downloads,
                            by: \.scanIndex
                        )
                        for (s, dls) in byScan {
                            self.dataRecorder?
                                .updateExposureTimestamps(
                                    scanIndex: s,
                                    downloads: dls
                                )
                        }
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
                insta360Results: [],
                frameAgeMs: frameAgeMs,
                trajectoryDeltaMs: trajDeltaMs
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
            var manualMotion: (peakDps: Double, meanDps: Double)?
            let manualCams = enabledCameras
            if !manualCams.isEmpty {
                if smoothedAngularRateDps >= 2.0 {
                    logMessage("waiting for steady…")
                    x5HoldSteady = true
                    let waitStart = ContinuousClock.now
                    while smoothedAngularRateDps >= 2.0 {
                        let elapsed = ContinuousClock.now
                            - waitStart
                        if elapsed > .seconds(2) { break }
                        try? await Task.sleep(
                            for: .milliseconds(50)
                        )
                    }
                    if smoothedAngularRateDps < 2.0 {
                        logMessage("steady — firing X5")
                    } else {
                        logMessage(String(
                            format: "timeout at %.0f°/s"
                                + " — firing X5",
                            smoothedAngularRateDps
                        ))
                    }
                }
                startX5HoldWindow()
                let cmdTime = ProcessInfo.processInfo
                    .systemUptime
                let motionTask = Task {
                    @MainActor [weak self] in
                    await self?.recordMotionWindow(
                        scanIndex: currentScan,
                        startTime: cmdTime
                    )
                }
                // 60 s, as in auto capture: the X6 can send the photo
                // URI in a push well after the takePicture reply, and a
                // push after the timeout is dropped.
                insta360Results = await insta360Manager
                    .captureWith(
                        cameras: manualCams,
                        arkitTimestamp: arkitFrame.timestamp,
                        scanIndex: currentScan,
                        timeout: 60
                    )
                let motion = await motionTask.value
                    ?? (peakDps: 0, meanDps: 0)
                manualMotion = motion
                logMessage(String(
                    format: "X5 scan#%d"
                        + " peak=%.1f°/s"
                        + " mean=%.1f°/s",
                    currentScan,
                    motion.peakDps,
                    motion.meanDps
                ))
            } else {
                insta360Results = []
            }

            dataRecorder?.saveScan(
                scanIndex: currentScan,
                arkitFrame: arkitFrame,
                insta360Results: insta360Results,
                frameAgeMs: frameAgeMs,
                trajectoryDeltaMs: trajDeltaMs
            )
            // updateInsta360Captures edits pose.json, so it has to run
            // after saveScan writes the file. Called before, it found
            // no file and the shot's motion values were lost.
            if let m = manualMotion {
                dataRecorder?.updateInsta360Captures(
                    scanIndex: currentScan,
                    results: insta360Results,
                    peakAngularRateDps: m.peakDps,
                    meanAngularRateDps: m.meanDps
                )
            }

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
                // Record each shot's EXIF time, as the auto path does.
                let byScan = Dictionary(
                    grouping: downloads, by: \.scanIndex)
                for (s, dls) in byScan {
                    dataRecorder?.updateExposureTimestamps(
                        scanIndex: s, downloads: dls)
                }
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
    /// True while captureScan() runs, so teardown can wait for it.
    private var scanInProgress = false
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

        let task = autoCaptureTask
        autoCaptureTask = nil
        task?.cancel()
        _ = await task?.value

        // Let an X5 shot that is already in flight finish and download.
        // Cancelling it made attemptTakePicture ignore the camera's
        // reply, so the session's last shot was never queued or
        // downloaded. Its capture and download have their own timeouts.
        let bgTask = insta360BackgroundTask
        insta360BackgroundTask = nil
        if bgTask != nil {
            logMessage("STOP: waiting for the last X5 shot")
        }
        _ = await bgTask?.value

        // A manual scan can also be waiting for the X5's reply;
        // clearCaptureState() below would drop it. Wait up to 40 s.
        var waits = 0
        while scanInProgress, waits < 400 {
            try? await Task.sleep(for: .milliseconds(100))
            waits += 1
        }

        await insta360Manager.clearCaptureState()
        logMessage("STOP: task done")
    }

    private static let buildTag = "12h"

    func endSession() async {
        guard isSessionActive, !isEndingSession else { return }
        isEndingSession = true
        isReadyToCapture = false
        diskSpaceTimer?.invalidate()
        diskSpaceTimer = nil

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
            insta360Manager.saveClockOffsets(
                to: dir, phase: "start"
            )
            var downloads =
                await insta360Manager.downloadAllPending(
                    into: dir
                )
            // Failed downloads are put back in the queue. The camera
            // may still be saving the last shot, so wait and retry once.
            if await insta360Manager.pendingDownloadCount() > 0 {
                logMessage("retrying X5 downloads")
                try? await Task.sleep(for: .seconds(3))
                downloads += await insta360Manager
                    .downloadAllPending(into: dir)
            }
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

    private nonisolated static let thumbContext = CIContext()

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
