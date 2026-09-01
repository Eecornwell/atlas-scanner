import Foundation
import Combine
import ARKit

/// Orchestrates the full capture session: ARKit tracking, Insta360 cameras, and data recording.
@MainActor
final class CaptureSessionManager: ObservableObject {
    @Published var isSessionActive = false
    @Published var isReadyToCapture = false
    @Published var scanCount = 0
    @Published var connectedCameraCount = 0
    @Published var sessionDirectory: URL?
    @Published var showExportSheet = false

    /// ARKit capture — exposed for CalibrationView.
    let arkitCapture = ARKitCapture()
    private let insta360Manager = Insta360CaptureManager()
    private let maskManager = MaskManager()
    private var dataRecorder: DataRecorder?
    private var trajectoryRecorder: TrajectoryRecorder?

    /// Last downloaded Insta360 ERP — exposed for CalibrationView.
    @Published var lastInstaERP: UIImage?

    func startSession() async {
        let sessionDir = SessionDirectory.create()
        dataRecorder = DataRecorder(sessionDirectory: sessionDir)
        sessionDirectory = sessionDir
        trajectoryRecorder = TrajectoryRecorder()

        arkitCapture.trajectoryRecorder = trajectoryRecorder
        arkitCapture.start()

        await insta360Manager.discoverAndConnect()
        connectedCameraCount = insta360Manager.connectedCameras.count

        maskManager.loadMasks(for: insta360Manager.connectedCameras, sessionDirectory: sessionDir)

        isSessionActive = true
        isReadyToCapture = true
        scanCount = 0
    }

    func captureScan() async {
        guard isSessionActive, isReadyToCapture else { return }
        isReadyToCapture = false
        defer { isReadyToCapture = true }

        guard let arkitFrame = arkitCapture.captureCurrentFrame() else { return }

        trajectoryRecorder?.recordPose(
            timestamp: arkitFrame.timestamp,
            pose: arkitFrame.pose
        )

        let insta360Results = await insta360Manager.captureAll(
            arkitTimestamp: arkitFrame.timestamp,
            scanIndex: scanCount
        )

        await dataRecorder?.saveScan(
            scanIndex: scanCount,
            arkitFrame: arkitFrame,
            insta360Results: insta360Results
        )

        // Keep the most recent Insta360 ERP available for calibration
        if let firstResult = insta360Results.first,
           let erpURL = dataRecorder?.erpURL(cameraId: firstResult.cameraId, scanIndex: scanCount),
           let data = try? Data(contentsOf: erpURL),
           let img = UIImage(data: data) {
            lastInstaERP = img
        }

        scanCount += 1
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
            try? exporter.export(scans: scans, cameraConfig: config)
        }

        isSessionActive = false
        connectedCameraCount = 0
        showExportSheet = true
    }
}
