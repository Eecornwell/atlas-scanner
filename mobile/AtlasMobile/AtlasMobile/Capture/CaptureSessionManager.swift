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

    /// ARKit capture — exposed for CalibrationView.
    let arkitCapture = ARKitCapture()
    private var insta360Manager = Insta360CaptureManager()
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
    @Published var useInsta360 = true

    @Published private(set) var isEndingSession = false
    @Published var teardownStage: String = ""
    @Published var captureTiming: String = ""
    private var connectTask: Task<Void, Never>?
    private var insta360CaptureTask: Task<Void, Never>?

    init() {
        arkitCapture.$trackingState
            .receive(on: DispatchQueue.main)
            .assign(to: &$trackingState)
        arkitCapture.$depthOverlayImage
            .receive(on: DispatchQueue.main)
            .assign(to: &$depthOverlayImage)
    }

    func startSession() async {
        guard !isSessionActive, !isEndingSession else { return }

        showExportSheet = false
        exportError = nil
        lastCapturedThumbnail = nil
        captureTiming = "[\(Self.buildTag)]"
        teardownStage = ""

        insta360Manager = Insta360CaptureManager()
        connectionLog = []
        insta360Manager.onLogMessage = { [weak self] msg in
            Task { @MainActor in
                self?.connectionLog.append(msg)
            }
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
                let cameras = await manager.connectedCamerasCopy()
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

        lastCapturedThumbnail = thumbnailFromPixelBuffer(
            arkitFrame.capturedImage
        )
        thumbnailVersion += 1
        if !fast {
            captureFlash = true
        }
        AudioServicesPlaySystemSound(1108)
        UIImpactFeedbackGenerator(style: .medium).impactOccurred()

        guard isSessionActive, !isEndingSession else { return }

        if fast {
            if insta360CaptureTask == nil,
               connectedCameraCount > 0,
               useInsta360
            {
                let manager = insta360Manager
                insta360CaptureTask = Task { [weak self] in
                    while let self,
                          self.isAutoCapturing,
                          self.isSessionActive,
                          !self.isEndingSession,
                          self.useInsta360
                    {
                        let idx = max(self.scanCount - 1, 0)
                        _ = await manager.captureAll(
                            arkitTimestamp: CACurrentMediaTime(),
                            scanIndex: idx,
                            timeout: 30
                        )
                        try? await Task.sleep(
                            nanoseconds: 500_000_000
                        )
                    }
                    self?.insta360CaptureTask = nil
                }
            }

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
            captureTiming = "[\(Self.buildTag)] #\(currentScan)"
                + " \(ms(t1 - t0))ms"
        } else {
            let insta360Results: [Insta360CaptureResult]
            if useInsta360, connectedCameraCount > 0 {
                insta360Results = await insta360Manager.captureAll(
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
        guard let first = downloads.first,
              let data = try? Data(contentsOf: first.localURL),
              let img = UIImage(data: data)
        else {
            if !downloads.isEmpty {
                print("[Capture] Insta360 download:"
                    + " \(downloads.count) files")
            }
            return
        }
        let maxW = 1024.0
        if img.size.width > maxW {
            let scale = maxW / img.size.width
            let sz = CGSize(
                width: maxW,
                height: img.size.height * scale
            )
            let renderer = UIGraphicsImageRenderer(size: sz)
            self.lastInstaERP = renderer.image { _ in
                img.draw(in: CGRect(origin: .zero, size: sz))
            }
        } else {
            self.lastInstaERP = img
        }
        self.thumbnailVersion += 1
    }

    private var autoCaptureTask: Task<Void, Never>?

    private static let autoCaptureCooldown: UInt64 = 2_500_000_000

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
        isAutoCapturing = false
        let task = autoCaptureTask
        autoCaptureTask = nil
        task?.cancel()
        _ = await task?.value
    }

    private static let buildTag = "10d"

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
        if let task = insta360CaptureTask {
            teardownStage = "1b-waitInsta"
            _ = await task.value
            insta360CaptureTask = nil
        }
        connectTask?.cancel()
        connectTask = nil
        insta360Manager.onCameraDisconnected = nil

        teardownStage = "2-stopARKit"
        arkitCapture.stop()

        let dir = sessionDirectory
        let recorder = dataRecorder
        let manager = insta360Manager

        teardownStage = "3-downloading"
        if let dir {
            manager.saveClockOffsets(to: dir)
            let downloads = await manager.downloadAllPending(
                into: dir
            )
            await recorder?.saveDownloadedMedia(downloads)
        }

        teardownStage = "4-disconnect"
        await manager.disconnect()
        connectedCameraCount = 0

        teardownStage = "5-trajectory"
        await recorder?.saveTrajectory(trajectoryRecorder?.export())

        teardownStage = "6-export"
        await Task.yield()
        if let dir, let recorder {
            let config = manager.cameraConfig
            let scans = recorder.buildExportData(sessionDirectory: dir)
            let exporter = COLMAPExporter(sessionDirectory: dir)
            do {
                try exporter.export(scans: scans, cameraConfig: config)
            } catch {
                exportError = error.localizedDescription
            }
        }
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

    // MARK: - Calibration capture

    func triggerInsta360Capture() async -> Bool {
        let count = await insta360Manager.connectedCount()
        guard count > 0 else { return false }
        let results = await insta360Manager.captureAll(
            arkitTimestamp: CACurrentMediaTime(), scanIndex: -1,
            timeout: 30
        )
        let ok = results.contains { !$0.mediaIdentifier.isEmpty }
        if !ok {
            print("[Insta360] capture returned no media URI"
                + " (timeout or camera not ready)")
        }
        return ok
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
        lastInstaERP = img
        return img
    }

    // MARK: - Insta360 retry

    func retryInsta360Connection() {
        let configuredCount = insta360Manager.cameraConfig.cameras.count
        guard configuredCount > 0 else { return }
        cameraStatus = "Retrying \(configuredCount) camera\(configuredCount == 1 ? "" : "s")…"
        connectionLog.append("--- Retry started ---")

        let manager = insta360Manager
        Task { [weak self] in
            await manager.retryConnection()
            guard let self else { return }
            let count = await manager.connectedCount()
            let total = configuredCount
            self.connectedCameraCount = count
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
                if count == 0 {
                    self.cameraStatus = "Camera disconnected"
                } else {
                    self.cameraStatus = "\(count)/\(total) cameras"
                }
            }
        }
    }

    private let ciContext = CIContext()

    func thumbnailFromPixelBuffer(_ buffer: CVPixelBuffer) -> UIImage? {
        autoreleasepool {
            let ciImage = CIImage(cvPixelBuffer: buffer)
            guard ciImage.extent.width > 0 else {
                return nil
            }
            let scale = 160.0 / ciImage.extent.width
            let scaled = ciImage.transformed(
                by: CGAffineTransform(scaleX: scale, y: scale)
            )
            guard let cgImage = ciContext.createCGImage(
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
