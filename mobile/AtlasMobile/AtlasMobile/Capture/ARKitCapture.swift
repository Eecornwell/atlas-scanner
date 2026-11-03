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
    @Published var speedMps: Double = 0
    @Published var angularRateDps: Double = 0
    @Published var smoothedAngularRateDps: Double = 0

    var showDepthOverlay = false
    var showMesh = false {
        didSet { updateSceneReconstruction() }
    }

    /// Injected by CaptureSessionManager so continuous poses feed the shared recorder.
    weak var trajectoryRecorder: TrajectoryRecorder?

    private var lastDepthUpdate: TimeInterval = 0
    private var lastSpeedPose: simd_float4x4?
    private var lastSpeedTimestamp: TimeInterval = 0
    private var angularRateHistory: [(time: TimeInterval, dps: Double)] = []

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
            smoothedDepthMap: smoothedDepth,
            trackingState: Self.trackingLabel(
                frame.camera.trackingState)
        )
    }

    /// Tracking state as saved in pose.json.
    static func trackingLabel(
        _ state: ARCamera.TrackingState
    ) -> String {
        switch state {
        case .normal:
            return "normal"
        case .notAvailable:
            return "notAvailable"
        case .limited(let reason):
            switch reason {
            case .initializing:
                return "limited:initializing"
            case .excessiveMotion:
                return "limited:excessiveMotion"
            case .insufficientFeatures:
                return "limited:insufficientFeatures"
            case .relocalizing:
                return "limited:relocalizing"
            @unknown default:
                return "limited:unknown"
            }
        @unknown default:
            return "unknown"
        }
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

        let dt = frame.timestamp - lastSpeedTimestamp
        if dt >= 0.2 {
            if let prev = lastSpeedPose {
                let cur = frame.camera.transform
                let dx = cur.columns.3 - prev.columns.3
                let dist = sqrt(dx.x * dx.x + dx.y * dx.y + dx.z * dx.z)
                let speed = Double(dist) / dt

                let R_prev = simd_float3x3(
                    SIMD3(prev.columns.0.x, prev.columns.0.y, prev.columns.0.z),
                    SIMD3(prev.columns.1.x, prev.columns.1.y, prev.columns.1.z),
                    SIMD3(prev.columns.2.x, prev.columns.2.y, prev.columns.2.z)
                )
                let R_cur = simd_float3x3(
                    SIMD3(cur.columns.0.x, cur.columns.0.y, cur.columns.0.z),
                    SIMD3(cur.columns.1.x, cur.columns.1.y, cur.columns.1.z),
                    SIMD3(cur.columns.2.x, cur.columns.2.y, cur.columns.2.z)
                )
                let R_delta = R_cur * R_prev.transpose
                let q = simd_quatf(R_delta)
                let angleRad = 2.0 * acos(min(abs(q.real), 1.0))
                let rateDps = Double(angleRad) * 180.0
                    / (.pi * dt)
                let now = frame.timestamp
                angularRateHistory.append(
                    (time: now, dps: rateDps)
                )
                let cutoff = now - 0.5
                angularRateHistory.removeAll {
                    $0.time < cutoff
                }
                let smoothed = angularRateHistory
                    .map(\.dps)
                    .reduce(0, +)
                    / Double(
                        max(angularRateHistory.count, 1)
                    )

                DispatchQueue.main.async {
                    self.speedMps = speed
                    self.angularRateDps = rateDps
                    self.smoothedAngularRateDps = smoothed
                }
            }
            lastSpeedPose = frame.camera.transform
            lastSpeedTimestamp = frame.timestamp
        }

        let wantsOverlay = showDepthOverlay
        if wantsOverlay, frame.timestamp - lastDepthUpdate > 0.033 {
            lastDepthUpdate = frame.timestamp
            if let depth = frame.sceneDepth?.depthMap {
                let image = colorizeDepthMap(depth, orientation: .portrait)
                DispatchQueue.main.async { self.depthOverlayImage = image }
            }
        } else if !wantsOverlay {
            DispatchQueue.main.async {
                if self.depthOverlayImage != nil {
                    self.depthOverlayImage = nil
                }
            }
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
