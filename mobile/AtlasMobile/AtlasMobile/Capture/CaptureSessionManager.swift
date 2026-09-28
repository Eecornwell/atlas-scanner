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
    /// Live connection diagnostic log for Insta360 cameras.
    @Published var connectionLog: [String] = []

    init() {
        arkitCapture.$trackingState
            .receive(on: DispatchQueue.main)
            .assign(to: &$trackingState)
        arkitCapture.$depthOverlayImage
            .receive(on: DispatchQueue.main)
            .assign(to: &$depthOverlayImage)
    }

    func startSession() async {
        showExportSheet = false
        exportError = nil
        lastCapturedThumbnail = nil

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
            Task { [insta360Manager, maskManager] in
                await insta360Manager.discoverAndConnect()
                let count = insta360Manager.connectedCameras.count
                await MainActor.run {
                    self.connectedCameraCount = count
                    if count == configuredCount {
                        self.cameraStatus = "\(count) camera\(count == 1 ? "" : "s") connected"
                    } else if count > 0 {
                        self.cameraStatus = "\(count)/\(configuredCount) cameras connected"
                    } else {
                        self.cameraStatus = "No cameras found"
                    }
                }
                maskManager.loadMasks(
                    for: insta360Manager.connectedCameras,
                    sessionDirectory: sessionDir
                )
            }
        }
    }

    func captureScan() async {
        guard isSessionActive, isReadyToCapture else { return }
        isReadyToCapture = false
        defer { isReadyToCapture = true }

        guard let arkitFrame = arkitCapture.captureCurrentFrame() else { return }
        let currentScan = scanCount
        scanCount += 1

        // UI feedback immediately
        captureFlash = true
        AudioServicesPlaySystemSound(1108)
        UIImpactFeedbackGenerator(style: .medium).impactOccurred()
        lastCapturedThumbnail = thumbnailFromPixelBuffer(arkitFrame.capturedImage)

        trajectoryRecorder?.recordPose(
            timestamp: arkitFrame.timestamp,
            pose: arkitFrame.pose
        )

        // Await Insta360 capture — blocks until camera confirms the shot
        let insta360Results = await insta360Manager.captureAll(
            arkitTimestamp: arkitFrame.timestamp,
            scanIndex: currentScan
        )

        // Save iPhone data in background
        let recorder = dataRecorder
        Task {
            await recorder?.saveScan(
                scanIndex: currentScan,
                arkitFrame: arkitFrame,
                insta360Results: insta360Results
            )
        }

        // Download 360 image after capture completes
        if !insta360Results.isEmpty, let dir = sessionDirectory {
            let mgr = insta360Manager
            Task {
                let downloads = await mgr.downloadAllPending(into: dir)
                if let first = downloads.first,
                   let data = try? Data(contentsOf: first.localURL),
                   let img = UIImage(data: data) {
                    self.lastInstaERP = img
                }
            }
        }
    }

    func endSession() async {
        isReadyToCapture = false

        if let dir = sessionDirectory {
            insta360Manager.saveClockOffsets(to: dir)
            let downloads = await insta360Manager.downloadAllPending(into: dir)
            await dataRecorder?.saveDownloadedMedia(downloads)
        }

        await dataRecorder?.saveTrajectory(trajectoryRecorder?.export())

        arkitCapture.stop()
        await insta360Manager.disconnect()

        if let dir = sessionDirectory, let recorder = dataRecorder {
            let config = insta360Manager.cameraConfig
            let scans = recorder.buildExportData(sessionDirectory: dir)
            let exporter = COLMAPExporter(sessionDirectory: dir)
            do {
                try exporter.export(scans: scans, cameraConfig: config)
            } catch {
                exportError = error.localizedDescription
            }
        }

        isSessionActive = false
        connectedCameraCount = 0
        showExportSheet = true
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

    // MARK: - Insta360 retry

    func retryInsta360Connection() {
        let configuredCount = insta360Manager.cameraConfig.cameras.count
        guard configuredCount > 0 else { return }
        cameraStatus = "Retrying \(configuredCount) camera\(configuredCount == 1 ? "" : "s")…"
        connectionLog.append("--- Retry started ---")

        Task {
            await insta360Manager.retryConnection()
            let count = insta360Manager.connectedCameras.count
            let total = configuredCount
            await MainActor.run {
                self.connectedCameraCount = count
                if count == total {
                    self.cameraStatus = "\(count) camera\(count == 1 ? "" : "s") connected"
                } else if count > 0 {
                    self.cameraStatus = "\(count)/\(total) cameras connected"
                } else {
                    self.cameraStatus = "No cameras found"
                }
            }
        }
    }

    private func setupDisconnectHandler() {
        insta360Manager.onCameraDisconnected = { [weak self] cameraId in
            guard let self else { return }
            let count = self.insta360Manager.connectedCameras.count
            let total = self.insta360Manager.cameraConfig.cameras.count
            self.connectedCameraCount = count
            if count == 0 {
                self.cameraStatus = "Camera disconnected"
            } else {
                self.cameraStatus = "\(count)/\(total) cameras"
            }
        }
    }

    private let ciContext = CIContext()

    private func thumbnailFromPixelBuffer(_ buffer: CVPixelBuffer) -> UIImage? {
        let ciImage = CIImage(cvPixelBuffer: buffer)
        let context = ciContext
        let scale = 120.0 / ciImage.extent.height
        let scaled = ciImage.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
        guard let cgImage = context.createCGImage(scaled, from: scaled.extent) else {
            return nil
        }
        return UIImage(cgImage: cgImage)
    }
}
