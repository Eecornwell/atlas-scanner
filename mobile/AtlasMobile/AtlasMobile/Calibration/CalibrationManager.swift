import Foundation
import simd
import ARKit
import UIKit

/// Orchestrates on-device Insta360 ↔ iPhone extrinsic calibration.
///
/// Flow:
///   1. User provides physical seed (RPY + XYZ from mount measurements)
///   2. addFrame() — stores ARKit depth + Insta360 ERP pairs
///   3. optimize() — runs KAZE+FLANN matching then Nelder-Mead 6-DOF refinement
///   4. overlayImage — visual verification composite
///   5. save() — writes refined extrinsic to multi_camera.yaml in Documents
@MainActor
final class CalibrationManager: ObservableObject {

    @Published var state: CalibrationState = .idle
    @Published var overlayImage: UIImage?
    @Published var refinedRPY: SIMD3<Float> = .zero
    @Published var refinedXYZ: SIMD3<Float> = .zero
    @Published var reprojectionError: Float = 0
    @Published var matchCount: Int = 0

    private var calibrationFrames: [CalibrationFrame] = []
    private let cameraId: String
    private let matcher = FeatureMatcher()

    init(cameraId: String = "insta360_primary") {
        self.cameraId = cameraId
    }

    // MARK: - Public API

    func addFrame(_ frame: CalibrationFrame) {
        calibrationFrames.append(frame)
        state = .framesCollected(calibrationFrames.count)
    }

    func clearFrames() {
        calibrationFrames = []
        matchCount = 0
        state = .idle
    }

    func optimize(seed: RigidTransform) async {
        guard !calibrationFrames.isEmpty else { return }
        state = .optimizing

        let frames = calibrationFrames

        let result: (RigidTransform, Float) = await Task.detached(priority: .userInitiated) {
            // Build LiDAR intensity ERP from first frame using seed
            guard let lidarERP = LiDARERPRenderer.render(
                frame: frames[0], extrinsic: seed
            ) else { return (seed, Float.infinity) }

            // KAZE + FLANN matching
            let fm = FeatureMatcher()
            guard fm.setReference(lidarERP) else { return (seed, Float.infinity) }
            guard let rawPairs = fm.match(frames[0].erpImage) else { return (seed, Float.infinity) }

            var pairs = [MatchedPair]()
            for v in rawPairs {
                var p = MatchedPair(lidarPt: .zero, instaPt: .zero)
                v.getValue(&p)
                pairs.append(p)
            }

            let erpW = Int(frames[0].erpImage.size.width)
            let erpH = Int(frames[0].erpImage.size.height)
            let lidarSize = lidarERP.size

            let samples = ExtrinsicOptimizer.buildSamples(
                pairs: pairs,
                frame: frames[0],
                lidarERPSize: lidarSize,
                instaERPSize: frames[0].erpImage.size
            )

            return ExtrinsicOptimizer.optimize(
                samples: samples,
                seed: seed,
                erpW: erpW,
                erpH: erpH
            )
        }.value

        let (refined, cost) = result
        refinedRPY = SIMD3(Float(refined.roll), Float(refined.pitch), Float(refined.yaw))
        refinedXYZ = SIMD3(Float(refined.x), Float(refined.y), Float(refined.z))
        reprojectionError = cost
        matchCount = Int(matcher.matchCount)

        if let first = calibrationFrames.first {
            overlayImage = CalibrationOverlayRenderer.render(
                frame: first, extrinsic: refined
            )
        }
        state = .done(refined)
    }

    /// Saves the refined extrinsic to Documents/atlas_sessions/multi_camera.yaml.
    func save(refined: RigidTransform) throws {
        var config = MultiCameraConfig.loadFromDeviceOrBundle()

        // Replace the extrinsic for the matching camera ID
        let updated = config.cameras.map { cam -> CameraConfig in
            guard cam.id == cameraId else { return cam }
            return CameraConfig(
                id: cam.id, model: cam.model, serial: cam.serial,
                extrinsic: refined,
                mask: cam.mask, faceCount: cam.faceCount, tileFov: cam.tileFov
            )
        }
        config = MultiCameraConfig(cameras: updated, iphone: config.iphone)
        try config.saveToDocuments()
        state = .saved
    }

    // MARK: - State

    enum CalibrationState: Equatable {
        case idle
        case framesCollected(Int)
        case optimizing
        case done(RigidTransform)
        case saved

        static func == (lhs: CalibrationState, rhs: CalibrationState) -> Bool {
            switch (lhs, rhs) {
            case (.idle, .idle), (.optimizing, .optimizing), (.saved, .saved): return true
            case (.framesCollected(let a), .framesCollected(let b)): return a == b
            case (.done(let a), .done(let b)):
                return a.roll == b.roll && a.pitch == b.pitch && a.yaw == b.yaw
                    && a.x == b.x && a.y == b.y && a.z == b.z
            default: return false
            }
        }
    }
}

// MARK: - Data types

/// One calibration sample: ARKit depth + pose + Insta360 ERP image.
struct CalibrationFrame {
    let depth: [Float]          // 256×192 Float32, metres
    let depthW: Int             // 256
    let depthH: Int             // 192
    let intrinsics: simd_float3x3
    let imageW: Int
    let imageH: Int
    let erpImage: UIImage       // Insta360 equirectangular
}
