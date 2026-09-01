import ARKit
import Combine

/// Manages the ARKit session for LiDAR depth, RGB frames, and 6DoF pose tracking.
final class ARKitCapture: NSObject, ObservableObject {
    private var session: ARSession?
    private var configuration: ARWorldTrackingConfiguration?

    @Published var isRunning = false
    @Published var trackingState: ARCamera.TrackingState = .notAvailable

    /// Injected by CaptureSessionManager so continuous poses feed the shared recorder.
    weak var trajectoryRecorder: TrajectoryRecorder?

    func start() {
        let config = ARWorldTrackingConfiguration()
        config.frameSemantics = [.sceneDepth, .smoothedSceneDepth]

        let session = ARSession()
        session.delegate = self
        session.run(config)

        self.session = session
        self.configuration = config
        self.isRunning = true
    }

    func stop() {
        session?.pause()
        session = nil
        isRunning = false
    }

    /// Captures the current ARFrame with all sensor data.
    func captureCurrentFrame() -> ARKitFrameData? {
        guard let frame = session?.currentFrame else { return nil }

        let pose = frame.camera.transform
        let intrinsics = frame.camera.intrinsics
        let timestamp = frame.timestamp

        guard let depthMap = frame.sceneDepth?.depthMap,
              let confidenceMap = frame.sceneDepth?.confidenceMap else {
            return nil
        }

        let smoothedDepth = frame.smoothedSceneDepth?.depthMap

        return ARKitFrameData(
            timestamp: timestamp,
            pose: pose,
            intrinsics: intrinsics,
            imageResolution: frame.camera.imageResolution,
            capturedImage: frame.capturedImage,
            depthMap: depthMap,
            confidenceMap: confidenceMap,
            smoothedDepthMap: smoothedDepth
        )
    }
}

extension ARKitCapture: ARSessionDelegate {
    func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
        DispatchQueue.main.async {
            self.trackingState = camera.trackingState
        }
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        trajectoryRecorder?.recordPose(
            timestamp: frame.timestamp,
            pose: frame.camera.transform
        )
    }
}
