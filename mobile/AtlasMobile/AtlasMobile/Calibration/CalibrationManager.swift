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
    @Published var overlayVersion: Int = 0
    @Published var refinedRPY: SIMD3<Float> = .zero
    @Published var refinedXYZ: SIMD3<Float> = .zero
    @Published var reprojectionError: Float = 0
    @Published var matchCount: Int = 0
    @Published var frameDebugInfo: String = ""
    @Published var maskStatus: String = ""
    @Published var colorProfileStatus: String = ""
    @Published var colorProfile: ColorProfile?
    @Published var colorPreviewIphone: UIImage?
    @Published var colorPreviewOriginal: UIImage?
    @Published var colorPreviewCorrected: UIImage?
    @Published var featureRefineStatus: String = ""
    @Published var featureMatchCount: Int = 0
    @Published var featurePreviewIphone: UIImage?
    @Published var featurePreviewSlice: UIImage?

    private(set) var calibrationFrames: [CalibrationFrame] = []
    var lastFrame: CalibrationFrame? { calibrationFrames.last }
    private let cameraId: String

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

    @Published var optimizeError: String?

    func optimize(seed: RigidTransform) async {
        guard let frame = calibrationFrames.first else { return }
        state = .optimizing
        optimizeError = nil

        guard let iphoneImg = frame.iphoneImage else {
            optimizeError = "No iPhone image for calibration"
            state = .framesCollected(calibrationFrames.count)
            return
        }

        guard let erpCG = frame.erpImage.cgImage else {
            optimizeError = "Could not process Insta360 image"
            state = .framesCollected(calibrationFrames.count)
            return
        }

        let outW = 256
        let outH = 192
        let di = frame.depthIntrinsics()
        let fx = di.fx * Float(outW) / Float(frame.depthW)
        let fy = di.fy * Float(outH) / Float(frame.depthH)
        let cx = di.cx * Float(outW) / Float(frame.depthW)
        let cy = di.cy * Float(outH) / Float(frame.depthH)

        let erpW = erpCG.width
        let erpH = erpCG.height
        print("[Calib] photometric alignment: iPhone→\(outW)×\(outH),"
            + " ERP \(erpW)×\(erpH)")

        guard let iphoneGray = Self.resizeToGrayscale(
            iphoneImg, width: outW, height: outH
        ) else {
            optimizeError = "Failed to process iPhone image"
            state = .framesCollected(calibrationFrames.count)
            return
        }
        guard let erpGray = Self.imageToGrayscale(erpCG) else {
            optimizeError = "Failed to process ERP image"
            state = .framesCollected(calibrationFrames.count)
            return
        }

        let capturedSeed = seed
        let (refined, cost) = await Task.detached(
            priority: .userInitiated
        ) {
            ExtrinsicOptimizer.optimizePhotometric(
                iphoneGray: iphoneGray,
                outW: outW, outH: outH,
                erpGray: erpGray,
                erpW: erpW, erpH: erpH,
                fx: fx, fy: fy, cx: cx, cy: cy,
                seed: capturedSeed
            )
        }.value

        let dRoll  = refined.roll  - seed.roll
        let dPitch = refined.pitch - seed.pitch
        let dYaw   = refined.yaw   - seed.yaw
        print("[Calib] done: NCC=\(String(format: "%.4f", -cost))")
        print("[Calib] seed  RPY=(\(seed.roll),\(seed.pitch),\(seed.yaw))")
        print("[Calib] refined RPY=(\(refined.roll),\(refined.pitch),\(refined.yaw))")
        print("[Calib] delta RPY=(\(String(format: "%.2f", dRoll))°,"
            + " \(String(format: "%.2f", dPitch))°,"
            + " \(String(format: "%.2f", dYaw))°)")

        refinedRPY = SIMD3(
            Float(refined.roll), Float(refined.pitch), Float(refined.yaw)
        )
        refinedXYZ = SIMD3(Float(refined.x), Float(refined.y), Float(refined.z))
        reprojectionError = -cost
        matchCount = outW * outH

        let deltaStr = String(
            format: "Δ RPY=(%.2f°, %.2f°, %.2f°) NCC=%.4f",
            dRoll, dPitch, dYaw, -cost
        )
        frameDebugInfo = frame.debugDescription + "\n" + deltaStr
        overlayImage = CalibrationOverlayRenderer.render(
            frame: frame, extrinsic: refined
        )
        overlayVersion += 1
        state = .done(refined)
    }

    func computeColorProfile(
        extrinsic: RigidTransform
    ) {
        guard let frame = calibrationFrames.first,
              let iphoneImg = frame.iphoneImage else {
            colorProfileStatus = "Color: no iPhone image"
            return
        }

        let maskImage = Self.loadMaskImage(cameraId: cameraId)

        guard let profile = ColorProfileCalibrator.compute(
            iphoneImage: iphoneImg,
            erpImage: frame.erpImage,
            extrinsic: extrinsic,
            frame: frame,
            mask: maskImage
        ) else {
            colorProfileStatus = "Color: computation failed"
            return
        }

        colorProfile = profile
        ColorProfileCalibrator.save(profile)
        colorProfileStatus = String(
            format: "Color: L×%.2f  a%+.1f  b%+.1f",
            profile.lGain, profile.aOffset, profile.bOffset
        )

        let thumbW = 512.0

        if let iphone = iphoneImg.cgImage {
            let iScale = thumbW / CGFloat(iphone.width)
            let iSize = CGSize(
                width: thumbW,
                height: CGFloat(iphone.height) * iScale
            )
            let r = UIGraphicsImageRenderer(size: iSize)
            colorPreviewIphone = r.image { _ in
                iphoneImg.draw(
                    in: CGRect(origin: .zero, size: iSize)
                )
            }
        }

        let erpSize = frame.erpImage.size
        if erpSize.width > thumbW {
            let scale = thumbW / erpSize.width
            let sz = CGSize(
                width: thumbW,
                height: erpSize.height * scale
            )
            let renderer = UIGraphicsImageRenderer(size: sz)
            let small = renderer.image { _ in
                frame.erpImage.draw(
                    in: CGRect(origin: .zero, size: sz)
                )
            }
            colorPreviewOriginal = small
            colorPreviewCorrected = ColorProfileCalibrator.apply(
                profile, to: small
            )
        } else {
            colorPreviewOriginal = frame.erpImage
            colorPreviewCorrected = ColorProfileCalibrator.apply(
                profile, to: frame.erpImage
            )
        }
    }

    func refineWithFeatures(
        extrinsic: RigidTransform
    ) async {
        guard let frame = calibrationFrames.first,
              let iphoneImg = frame.iphoneImage else {
            featureRefineStatus = "Features: no iPhone image"
            return
        }

        featureRefineStatus = "Features: detecting…"
        let maskImage = Self.loadMaskImage(cameraId: cameraId)
        let seed = extrinsic

        // Color-correct the ERP before slicing so features match
        let erp: UIImage
        if let cp = colorProfile {
            erp = ColorProfileCalibrator.apply(cp, to: frame.erpImage)
            print("[FeatureCal] using color-corrected ERP")
        } else if let cp = ColorProfileCalibrator.load() {
            erp = ColorProfileCalibrator.apply(cp, to: frame.erpImage)
            print("[FeatureCal] loaded saved color profile")
        } else {
            erp = frame.erpImage
            print("[FeatureCal] no color profile — using raw ERP")
        }

        let result = await Task.detached(
            priority: .userInitiated
        ) {
            FeatureBasedCalibrator.calibrate(
                iphoneImage: iphoneImg,
                erpImage: erp,
                seed: seed,
                frame: frame,
                mask: maskImage
            )
        }.value

        // Always show previews so user can verify the slice
        featurePreviewIphone = result.iphonePreview
        featurePreviewSlice = result.slicePreview

        guard result.matchCount > 0 else {
            featureRefineStatus =
                "Features: failed (too few matches — see previews)"
            return
        }

        featureMatchCount = result.matchCount
        featureRefineStatus = String(
            format: "Features: %d matches, cost %.1f"
                + " | %d/%d kps",
            result.matchCount, result.cost,
            result.iphoneKPCount, result.sliceKPCount
        )

        let refined = result.extrinsic
        refinedRPY = SIMD3(
            Float(refined.roll),
            Float(refined.pitch),
            Float(refined.yaw)
        )
        refinedXYZ = SIMD3(
            Float(refined.x),
            Float(refined.y),
            Float(refined.z)
        )
        reprojectionError = result.cost

        let dR = refined.roll - extrinsic.roll
        let dP = refined.pitch - extrinsic.pitch
        let dY = refined.yaw - extrinsic.yaw
        let deltaStr = String(
            format: "Feature Δ RPY=(%.2f°, %.2f°, %.2f°)",
            dR, dP, dY
        )
        frameDebugInfo += "\n" + deltaStr

        overlayImage = CalibrationOverlayRenderer.render(
            frame: frame, extrinsic: refined
        )
        overlayVersion += 1
        state = .done(refined)
    }

    private static func resizeToGrayscale(
        _ image: UIImage, width: Int, height: Int
    ) -> [UInt8]? {
        let size = CGSize(width: width, height: height)
        let fmt = UIGraphicsImageRendererFormat()
        fmt.scale = 1.0
        let renderer = UIGraphicsImageRenderer(size: size, format: fmt)
        let resized = renderer.image { _ in
            image.draw(in: CGRect(origin: .zero, size: size))
        }
        guard let cg = resized.cgImage else { return nil }
        return imageToGrayscale(cg)
    }

    private static func imageToGrayscale(
        _ cg: CGImage
    ) -> [UInt8]? {
        let w = cg.width, h = cg.height
        var gray = [UInt8](repeating: 0, count: w * h)
        guard let ctx = CGContext(
            data: &gray, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w,
            space: CGColorSpaceCreateDeviceGray(),
            bitmapInfo: CGImageAlphaInfo.none.rawValue
        ) else { return nil }
        ctx.draw(cg, in: CGRect(x: 0, y: 0, width: w, height: h))
        return gray
    }

    /// Expands masked pixels by `radius` in all directions so the hard
    /// mask boundary doesn't register as an edge during optimization.
    private static func dilateMask(
        _ mask: [Bool], w: Int, h: Int, radius: Int
    ) -> [Bool] {
        var dilated = mask
        for y in 0..<h {
            for x in 0..<w {
                guard mask[y * w + x] else { continue }
                let yLo = max(0, y - radius)
                let yHi = min(h - 1, y + radius)
                let xLo = max(0, x - radius)
                let xHi = min(w - 1, x + radius)
                for dy in yLo...yHi {
                    for dx in xLo...xHi {
                        dilated[dy * w + dx] = true
                    }
                }
            }
        }
        return dilated
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
                mask: cam.mask, faceCount: cam.faceCount, tileFov: cam.tileFov,
                enabled: cam.enabled
            )
        }
        config = MultiCameraConfig(cameras: updated, iphone: config.iphone)
        try config.saveToDocuments()
        state = .saved
    }

    static func loadMaskImage(cameraId: String) -> UIImage? {
        guard let dir = FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first else {
            print("[Calib] loadMask: no Documents dir")
            return nil
        }
        let url = dir
            .appendingPathComponent("atlas_sessions/masks")
            .appendingPathComponent("\(cameraId).png")
        let exists = FileManager.default.fileExists(atPath: url.path)
        print("[Calib] loadMask: \(url.path) exists=\(exists)")

        if !exists {
            let masksDir = dir.appendingPathComponent("atlas_sessions/masks")
            if let files = try? FileManager.default.contentsOfDirectory(
                at: masksDir, includingPropertiesForKeys: nil
            ) {
                print("[Calib] loadMask: masks dir contains \(files.map { $0.lastPathComponent })")
            } else {
                print("[Calib] loadMask: masks dir does not exist or is empty")
            }
            return nil
        }

        guard let data = try? Data(contentsOf: url) else {
            print("[Calib] loadMask: failed to read data from \(url.path)")
            return nil
        }
        print("[Calib] loadMask: read \(data.count) bytes")
        guard let img = UIImage(data: data) else {
            print("[Calib] loadMask: UIImage(data:) failed")
            return nil
        }
        print("[Calib] loadMask: loaded \(Int(img.size.width))x\(Int(img.size.height))")
        return img
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
    let imageW: Int             // CVPixelBufferGetWidth(capturedImage) — always landscape
    let imageH: Int             // CVPixelBufferGetHeight(capturedImage) — always landscape
    let imageResW: Int          // camera.imageResolution.width — may differ on portrait apps
    let imageResH: Int          // camera.imageResolution.height — may differ on portrait apps
    let erpImage: UIImage       // Insta360 equirectangular
    let iphoneImage: UIImage?   // iPhone RGB (landscape, for color calibration)

    struct DepthIntrinsics {
        let fx: Float
        let fy: Float
        let cx: Float
        let cy: Float
    }

    /// Intrinsics scaled to depth resolution.
    ///
    /// The depth buffer is always landscape (256×192). ARKit's
    /// intrinsics matrix matches the coordinate system of
    /// imageResolution, which may report portrait dimensions
    /// on portrait-only apps. When that happens, we rotate the
    /// intrinsics from portrait to landscape before scaling.
    func depthIntrinsics() -> DepthIntrinsics {
        let isPortrait = imageResW < imageResH

        if isPortrait {
            // Intrinsics are in portrait coords; rotate to landscape.
            // landscape X ← portrait Y, landscape Y ← reversed portrait X
            let scaleX = Float(depthW) / Float(imageW)
            let scaleY = Float(depthH) / Float(imageH)
            return DepthIntrinsics(
                fx: intrinsics[1][1] * scaleX,
                fy: intrinsics[0][0] * scaleY,
                cx: intrinsics[2][1] * scaleX,
                cy: (Float(imageResW) - 1 - intrinsics[2][0])
                    * Float(depthH) / Float(imageResW)
            )
        }

        let scaleX = Float(depthW) / Float(imageW)
        let scaleY = Float(depthH) / Float(imageH)
        return DepthIntrinsics(
            fx: intrinsics[0][0] * scaleX,
            fy: intrinsics[1][1] * scaleY,
            cx: intrinsics[2][0] * scaleX,
            cy: intrinsics[2][1] * scaleY
        )
    }

    var debugDescription: String {
        let di = depthIntrinsics()
        let portrait = imageResW < imageResH ? "YES" : "no"
        return "depth=\(depthW)×\(depthH)"
            + " buf=\(imageW)×\(imageH)"
            + " res=\(imageResW)×\(imageResH)"
            + " portrait=\(portrait)"
            + " di(fx=\(String(format: "%.1f", di.fx))"
            + " fy=\(String(format: "%.1f", di.fy))"
            + " cx=\(String(format: "%.1f", di.cx))"
            + " cy=\(String(format: "%.1f", di.cy)))"
    }
}

