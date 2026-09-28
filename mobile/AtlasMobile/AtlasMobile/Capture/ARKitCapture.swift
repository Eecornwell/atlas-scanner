import ARKit
import Combine
import UIKit

/// Manages the ARKit session for LiDAR depth, RGB frames, and 6DoF pose tracking.
final class ARKitCapture: NSObject, ObservableObject {
    private(set) var session: ARSession?
    private var configuration: ARWorldTrackingConfiguration?

    @Published var isRunning = false
    @Published var trackingState: ARCamera.TrackingState = .notAvailable
    @Published var depthOverlayImage: UIImage?

    var showDepthOverlay = false
    var showMesh = false {
        didSet { updateSceneReconstruction() }
    }

    /// Injected by CaptureSessionManager so continuous poses feed the shared recorder.
    weak var trajectoryRecorder: TrajectoryRecorder?

    private var lastDepthUpdate: TimeInterval = 0

    func start() {
        let config = ARWorldTrackingConfiguration()
        config.frameSemantics = [.sceneDepth, .smoothedSceneDepth]
        if showMesh && ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
        }

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
        depthOverlayImage = nil
    }

    private func updateSceneReconstruction() {
        guard let config = configuration, let session = session else { return }
        if showMesh && ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
        } else {
            config.sceneReconstruction = []
        }
        session.run(config)
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

        if showDepthOverlay, frame.timestamp - lastDepthUpdate > 0.1 {
            lastDepthUpdate = frame.timestamp
            if let depth = frame.sceneDepth?.depthMap {
                let orientation = UIApplication.shared.connectedScenes
                    .compactMap { $0 as? UIWindowScene }.first?
                    .interfaceOrientation ?? .portrait
                let image = colorizeDepthMap(depth, orientation: orientation)
                DispatchQueue.main.async { self.depthOverlayImage = image }
            }
        } else if !showDepthOverlay && depthOverlayImage != nil {
            DispatchQueue.main.async { self.depthOverlayImage = nil }
        }
    }

    private func colorizeDepthMap(
        _ depthMap: CVPixelBuffer,
        orientation: UIInterfaceOrientation
    ) -> UIImage? {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        let w = CVPixelBufferGetWidth(depthMap)
        let h = CVPixelBufferGetHeight(depthMap)
        guard let base = CVPixelBufferGetBaseAddress(depthMap) else { return nil }
        let floats = base.assumingMemoryBound(to: Float.self)

        var pixels = [UInt8](repeating: 0, count: w * h * 4)
        let maxDepth: Float = 5.0

        for i in 0..<(w * h) {
            let t = min(max(floats[i] / maxDepth, 0), 1)
            let (r, g, b) = Self.jetColor(t)
            pixels[i * 4] = r
            pixels[i * 4 + 1] = g
            pixels[i * 4 + 2] = b
            pixels[i * 4 + 3] = 160
        }

        let colorSpace = CGColorSpaceCreateDeviceRGB()
        guard let ctx = CGContext(
            data: &pixels, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: colorSpace,
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ), let cgImage = ctx.makeImage() else { return nil }

        let imageOrientation: UIImage.Orientation
        switch orientation {
        case .portrait:
            imageOrientation = .right
        case .portraitUpsideDown:
            imageOrientation = .left
        case .landscapeLeft:
            imageOrientation = .up
        case .landscapeRight:
            imageOrientation = .down
        default:
            imageOrientation = .right
        }
        return UIImage(cgImage: cgImage, scale: 1.0, orientation: imageOrientation)
    }

    private static func jetColor(_ t: Float) -> (UInt8, UInt8, UInt8) {
        let r: Float, g: Float, b: Float
        if t < 0.25 {
            r = 0; g = 4 * t; b = 1
        } else if t < 0.5 {
            r = 0; g = 1; b = 1 - 4 * (t - 0.25)
        } else if t < 0.75 {
            r = 4 * (t - 0.5); g = 1; b = 0
        } else {
            r = 1; g = 1 - 4 * (t - 0.75); b = 0
        }
        return (UInt8(r * 255), UInt8(g * 255), UInt8(b * 255))
    }
}
