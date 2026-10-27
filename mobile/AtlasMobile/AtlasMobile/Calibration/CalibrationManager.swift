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
///   4. overlayPair — side-by-side verification images
///   5. save() — writes refined extrinsic to multi_camera.yaml in Documents
@MainActor
final class CalibrationManager: ObservableObject {

    @Published var state: CalibrationState = .idle
    @Published var overlayPair: CalibrationOverlayRenderer.Pair?
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
    var cameraId: String

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

    private var bestFrameIndex: Int = 0

    func optimize(seed: RigidTransform) async {
        guard !calibrationFrames.isEmpty else { return }
        state = .optimizing
        optimizeError = nil

        let outW = 256
        let outH = 192
        let capturedSeed = seed

        var bestRefined: RigidTransform?
        var bestCost = Float.infinity
        var bestIdx = 0
        var bestFrame: CalibrationFrame?
        var validCount = 0

        for (idx, frame) in calibrationFrames.enumerated() {
            guard let iphoneImg = frame.iphoneImage,
                  let erpCG = frame.erpImage.cgImage
            else { continue }

            let di = frame.depthIntrinsics()
            let fx = di.fx * Float(outW) / Float(frame.depthW)
            let fy = di.fy * Float(outH) / Float(frame.depthH)
            let cx = di.cx * Float(outW) / Float(frame.depthW)
            let cy = di.cy * Float(outH) / Float(frame.depthH)
            let erpW = erpCG.width
            let erpH = erpCG.height

            guard let iphoneGray = Self.resizeToGrayscale(
                iphoneImg, width: outW, height: outH
            ),
                  let erpGray = Self.imageToGrayscale(erpCG)
            else { continue }

            validCount += 1
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

            print("[Calib] frame \(idx): NCC="
                + "\(String(format: "%.4f", -cost))")

            if cost < bestCost {
                bestCost = cost
                bestRefined = refined
                bestIdx = idx
                bestFrame = frame
            }
        }

        guard let refined = bestRefined,
              let frame = bestFrame
        else {
            optimizeError = "No valid frames for calibration"
            state = .framesCollected(calibrationFrames.count)
            return
        }

        bestFrameIndex = bestIdx

        let dRoll  = refined.roll  - seed.roll
        let dPitch = refined.pitch - seed.pitch
        let dYaw   = refined.yaw   - seed.yaw
        print("[Calib] best frame \(bestIdx)/\(validCount):"
            + " NCC=\(String(format: "%.4f", -bestCost))")
        print("[Calib] seed  RPY=(\(seed.roll),"
            + "\(seed.pitch),\(seed.yaw))")
        print("[Calib] refined RPY=(\(refined.roll),"
            + "\(refined.pitch),\(refined.yaw))")
        print("[Calib] delta RPY=("
            + "\(String(format: "%.2f", dRoll))°,"
            + " \(String(format: "%.2f", dPitch))°,"
            + " \(String(format: "%.2f", dYaw))°)")

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
        reprojectionError = Float(-bestCost)
        matchCount = outW * outH * validCount

        let deltaStr = String(
            format: "best %d/%d Δ RPY=(%.2f°, %.2f°, %.2f°)"
                + " NCC=%.4f",
            bestIdx + 1, validCount,
            dRoll, dPitch, dYaw, -bestCost
        )
        frameDebugInfo = frame.debugDescription + "\n" + deltaStr
        overlayPair = CalibrationOverlayRenderer.render(
            frame: frame, extrinsic: refined
        )
        overlayVersion += 1
        state = .done(refined)
    }

    func computeColorProfile(
        extrinsic: RigidTransform
    ) {
        let frameIdx = min(
            bestFrameIndex, calibrationFrames.count - 1
        )
        guard frameIdx >= 0,
              let iphoneImg = calibrationFrames[frameIdx]
                  .iphoneImage
        else {
            colorProfileStatus = "Color: no iPhone image"
            return
        }

        let frame = calibrationFrames[frameIdx]
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
        guard !calibrationFrames.isEmpty else {
            featureRefineStatus = "Features: no frames"
            return
        }

        featureRefineStatus = "Features: detecting…"
        let maskImage = Self.loadMaskImage(cameraId: cameraId)
        let seed = extrinsic

        let cp = colorProfile ?? ColorProfileCalibrator.load()
        if cp != nil {
            print("[FeatureCal] using color profile")
        }

        // Prepare per-frame ERP images (color-corrected)
        struct FrameInput {
            let index: Int
            let frame: CalibrationFrame
            let iphoneImage: UIImage
            let erpImage: UIImage
        }
        var inputs: [FrameInput] = []
        for (idx, frame) in calibrationFrames.enumerated()
        {
            guard let iImg = frame.iphoneImage
            else { continue }
            let erp: UIImage
            if let cp {
                erp = ColorProfileCalibrator.apply(
                    cp, to: frame.erpImage
                )
            } else {
                erp = frame.erpImage
            }
            inputs.append(FrameInput(
                index: idx, frame: frame,
                iphoneImage: iImg, erpImage: erp
            ))
        }

        guard !inputs.isEmpty else {
            featureRefineStatus =
                "Features: no valid frames"
            return
        }

        let erpW = Int(
            inputs[0].erpImage.size.width
        )
        let erpH = Int(
            inputs[0].erpImage.size.height
        )

        // ---------- Round 1: match all frames at seed ----------
        featureRefineStatus = "Features: round 1…"
        var allSamples:
            [ExtrinsicOptimizer.MatchSample] = []
        var bestMatchIdx = 0
        var bestMatchCount = 0
        var bestPreviewI: UIImage?
        var bestPreviewS: UIImage?

        for fi in inputs {
            let mr = await Task.detached(
                priority: .userInitiated
            ) {
                FeatureBasedCalibrator.matchFrame(
                    iphoneImage: fi.iphoneImage,
                    erpImage: fi.erpImage,
                    seed: seed,
                    frame: fi.frame,
                    mask: maskImage
                )
            }.value

            print("[FeatureCal] R1 frame \(fi.index):"
                + " \(mr.samples.count) samples,"
                + " \(mr.nccMatchCount) NCC,"
                + " \(mr.iphoneKPCount) kp")

            allSamples.append(contentsOf: mr.samples)

            if mr.samples.count > bestMatchCount {
                bestMatchCount = mr.samples.count
                bestMatchIdx = fi.index
                bestPreviewI = mr.iphonePreview
                bestPreviewS = mr.slicePreview
            }
        }

        print("[FeatureCal] R1 total: \(allSamples.count)"
            + " samples from \(inputs.count) frames")

        guard allSamples.count >= 8 else {
            featureRefineStatus = String(
                format: "Features: too few matches"
                    + " (%d across %d frames)",
                allSamples.count, inputs.count
            )
            return
        }

        // ---------- Joint optimization round 1 ----------
        var round1 = FeatureBasedCalibrator.optimizeJoint(
            samples: &allSamples,
            seed: seed,
            erpW: erpW, erpH: erpH
        )

        print("[FeatureCal] R1 result:"
            + " RPY=(\(String(format: "%.3f", round1.roll)),"
            + " \(String(format: "%.3f", round1.pitch)),"
            + " \(String(format: "%.3f", round1.yaw)))"
            + " inliers=\(allSamples.count)")

        // ---------- Round 2: re-slice at refined seed ----------
        featureRefineStatus = "Features: round 2…"
        var allSamples2:
            [ExtrinsicOptimizer.MatchSample] = []

        for fi in inputs {
            let refinedSeed = round1
            let mr = await Task.detached(
                priority: .userInitiated
            ) {
                FeatureBasedCalibrator.matchFrame(
                    iphoneImage: fi.iphoneImage,
                    erpImage: fi.erpImage,
                    seed: refinedSeed,
                    frame: fi.frame,
                    mask: maskImage
                )
            }.value

            print("[FeatureCal] R2 frame \(fi.index):"
                + " \(mr.samples.count) samples")

            allSamples2.append(contentsOf: mr.samples)

            if mr.samples.count > bestMatchCount {
                bestMatchCount = mr.samples.count
                bestMatchIdx = fi.index
                bestPreviewI = mr.iphonePreview
                bestPreviewS = mr.slicePreview
            }
        }

        print("[FeatureCal] R2 total:"
            + " \(allSamples2.count) samples")

        // ---------- Joint optimization round 2 ----------
        let refined: RigidTransform
        let finalSampleCount: Int
        if allSamples2.count >= 8 {
            refined = FeatureBasedCalibrator.optimizeJoint(
                samples: &allSamples2,
                seed: round1,
                erpW: erpW, erpH: erpH,
                finalIterations: 1500
            )
            finalSampleCount = allSamples2.count
        } else {
            refined = round1
            finalSampleCount = allSamples.count
        }

        print("[FeatureCal] final:"
            + " RPY=(\(String(format: "%.3f", refined.roll)),"
            + " \(String(format: "%.3f", refined.pitch)),"
            + " \(String(format: "%.3f", refined.yaw)))"
            + " inliers=\(finalSampleCount)")

        // ---------- Update UI ----------
        featurePreviewIphone = bestPreviewI
        featurePreviewSlice = bestPreviewS
        featureMatchCount = finalSampleCount

        let finalCost = FeatureBasedCalibrator.jointCost(
            samples: allSamples2.isEmpty
                ? allSamples : allSamples2,
            extrinsic: refined,
            erpW: erpW, erpH: erpH
        )
        featureRefineStatus = String(
            format: "Features: %d inliers, %d frames,"
                + " cost=%.1f",
            finalSampleCount, inputs.count, finalCost
        )

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
        reprojectionError = finalCost

        let dR = refined.roll - extrinsic.roll
        let dP = refined.pitch - extrinsic.pitch
        let dY = refined.yaw - extrinsic.yaw
        let deltaStr = String(
            format: "Joint %d frames, 2 rounds"
                + " Δ RPY=(%.2f°, %.2f°, %.2f°)"
                + " cost=%.1f",
            inputs.count, dR, dP, dY, finalCost
        )
        frameDebugInfo += "\n" + deltaStr

        let overlayFrame = calibrationFrames[
            bestMatchIdx
        ]
        overlayPair = CalibrationOverlayRenderer.render(
            frame: overlayFrame, extrinsic: refined
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

        var isOptimized: Bool {
            switch self {
            case .done, .saved: return true
            default: return false
            }
        }

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

