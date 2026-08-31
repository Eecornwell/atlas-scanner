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

    private let arkitCapture = ARKitCapture()
    private let insta360Manager = Insta360CaptureManager()
    private let maskManager = MaskManager()
    private var dataRecorder: DataRecorder?
    private var trajectoryRecorder: TrajectoryRecorder?

    func startSession() async {
        let sessionDir = SessionDirectory.create()
        dataRecorder = DataRecorder(sessionDirectory: sessionDir)
        trajectoryRecorder = TrajectoryRecorder()

        arkitCapture.start()

        await insta360Manager.discoverAndConnect()
        connectedCameraCount = insta360Manager.connectedCameras.count

        maskManager.loadMasks(for: insta360Manager.connectedCameras)

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
            arkitTimestamp: arkitFrame.timestamp
        )

        await dataRecorder?.saveScan(
            scanIndex: scanCount,
            arkitFrame: arkitFrame,
            insta360Results: insta360Results
        )

        scanCount += 1
    }

    func endSession() async {
        isReadyToCapture = false

        let downloads = await insta360Manager.downloadAllPending()
        await dataRecorder?.saveDownloadedMedia(downloads)

        await dataRecorder?.saveTrajectory(trajectoryRecorder?.export())

        arkitCapture.stop()
        await insta360Manager.disconnect()

        // TODO: Run on-device COLMAP export
        // await ExportManager.exportCOLMAP(sessionDirectory: dataRecorder.sessionDirectory)

        isSessionActive = false
        connectedCameraCount = 0
    }
}
