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

        print("[Calib] building LiDAR bearings from depth \(frame.depthW)x\(frame.depthH)")
        let bearings = ExtrinsicOptimizer.buildBearings(frame: frame)
        print("[Calib] \(bearings.count) valid bearings")

        guard !bearings.isEmpty else {
            optimizeError = "No valid depth points"
            state = .framesCollected(calibrationFrames.count)
            return
        }

        print("[Calib] computing Insta360 edge map")
        guard let edges = ExtrinsicOptimizer.sobelEdges(
            from: frame.erpImage
        ) else {
            optimizeError = "Could not process Insta360 image"
            state = .framesCollected(calibrationFrames.count)
            return
        }
        print("[Calib] edge map \(edges.w)x\(edges.h), running Nelder-Mead...")

        let edgeMap = edges.edges
        let edgeW = edges.w
        let edgeH = edges.h

        let (refined, cost) = await Task.detached(
            priority: .userInitiated
        ) {
            ExtrinsicOptimizer.optimizeEdgeBased(
                bearings: bearings,
                edgeMap: edgeMap,
                edgeW: edgeW,
                edgeH: edgeH,
                seed: seed
            )
        }.value

        print("[Calib] done: cost=\(cost)")

        refinedRPY = SIMD3(
            Float(refined.roll), Float(refined.pitch), Float(refined.yaw)
        )
        refinedXYZ = SIMD3(Float(refined.x), Float(refined.y), Float(refined.z))
        reprojectionError = cost
        matchCount = bearings.count

        frameDebugInfo = frame.debugDescription
        overlayImage = CalibrationOverlayRenderer.render(
            frame: frame, extrinsic: refined
        )
        overlayVersion += 1
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
    let imageW: Int             // CVPixelBufferGetWidth(capturedImage) — always landscape
    let imageH: Int             // CVPixelBufferGetHeight(capturedImage) — always landscape
    let imageResW: Int          // camera.imageResolution.width — may differ on portrait apps
    let imageResH: Int          // camera.imageResolution.height — may differ on portrait apps
    let erpImage: UIImage       // Insta360 equirectangular

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

// MARK: - Dual-fisheye → ERP stitcher

enum DualFisheyeStitcher {

    static func isDualFisheye(_ image: UIImage) -> Bool {
        guard let cg = image.cgImage else { return false }
        let ratio = Float(cg.width) / Float(cg.height)
        guard ratio > 1.8 && ratio < 2.2 else { return false }

        let checkW = 64, checkH = 32
        var pixels = [UInt8](repeating: 0, count: checkW * checkH * 4)
        guard let ctx = CGContext(
            data: &pixels, width: checkW, height: checkH,
            bitsPerComponent: 8, bytesPerRow: checkW * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return false }
        ctx.draw(cg, in: CGRect(x: 0, y: 0, width: checkW, height: checkH))

        var total: Float = 0
        var count = 0
        let patch = 4
        for (cx, cy) in [(0, 0), (checkW - patch, 0),
                          (0, checkH - patch), (checkW - patch, checkH - patch)] {
            for dy in 0..<patch {
                for dx in 0..<patch {
                    let off = ((cy + dy) * checkW + (cx + dx)) * 4
                    let luma = (Float(pixels[off]) + Float(pixels[off + 1])
                                + Float(pixels[off + 2])) / 3
                    total += luma
                    count += 1
                }
            }
        }
        return (total / Float(count)) < 15
    }

    static func stitch(
        _ dualFisheye: UIImage,
        outputWidth: Int = 2048
    ) -> UIImage? {
        guard let origCG = dualFisheye.cgImage else { return nil }
        let maxSrcW = 3072
        let cg: CGImage
        if origCG.width > maxSrcW {
            let scale = CGFloat(maxSrcW) / CGFloat(origCG.width)
            let newH = Int(CGFloat(origCG.height) * scale)
            let renderer = UIGraphicsImageRenderer(
                size: CGSize(width: maxSrcW, height: newH)
            )
            let downsampled = renderer.image { _ in
                UIImage(cgImage: origCG).draw(
                    in: CGRect(x: 0, y: 0, width: maxSrcW, height: newH)
                )
            }
            guard let dsCG = downsampled.cgImage else { return nil }
            cg = dsCG
        } else {
            cg = origCG
        }
        let srcW = cg.width, srcH = cg.height
        let halfW = srcW / 2

        var src = [UInt8](repeating: 0, count: srcW * srcH * 4)
        guard let srcCtx = CGContext(
            data: &src, width: srcW, height: srcH,
            bitsPerComponent: 8, bytesPerRow: srcW * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return nil }
        srcCtx.draw(cg, in: CGRect(x: 0, y: 0, width: srcW, height: srcH))

        let radius = Float(srcH) * 0.47
        let halfFov: Float = 100 * .pi / 180
        let focal = radius / halfFov

        let fCx = Float(halfW) * 0.5
        let fCy = Float(srcH) * 0.5
        let bCx = Float(halfW) + Float(halfW) * 0.5
        let bCy = Float(srcH) * 0.5

        let outW = outputWidth
        let outH = outputWidth / 2
        var out = [UInt8](repeating: 0, count: outW * outH * 4)

        let W = Float(outW), H = Float(outH)
        let blendLo: Float = 85 * .pi / 180
        let blendHi: Float = 95 * .pi / 180
        let srcStride = srcW * 4

        for ov in 0..<outH {
            let lat = Float.pi * (0.5 - Float(ov) / H)
            let cosLat = cos(lat)
            let sinLat = sin(lat)

            for ou in 0..<outW {
                let lon = 2 * Float.pi * (Float(ou) / W - 0.5)
                let bx = cosLat * sin(lon)
                let by = -sinLat
                let bz = cosLat * cos(lon)
                let rxy = sqrt(bx * bx + by * by)

                let tF = atan2(rxy, bz)
                let tB = atan2(rxy, -bz)

                let idx = (ov * outW + ou) * 4
                var r: Float, g: Float, b: Float

                if tF < blendLo {
                    (r, g, b) = sampleLens(
                        src: src, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy, theta: tF,
                        cx: fCx, cy: fCy, f: focal, rad: radius,
                        mirror: false
                    )
                } else if tB < blendLo {
                    (r, g, b) = sampleLens(
                        src: src, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy, theta: tB,
                        cx: bCx, cy: bCy, f: focal, rad: radius,
                        mirror: true
                    )
                } else {
                    let wF = max(0, blendHi - tF)
                    let wB = max(0, blendHi - tB)
                    let sum = wF + wB
                    let aF = sum > 0 ? wF / sum : 0.5

                    let (fr, fg, fb) = sampleLens(
                        src: src, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy, theta: tF,
                        cx: fCx, cy: fCy, f: focal, rad: radius,
                        mirror: false
                    )
                    let (br2, bg2, bb2) = sampleLens(
                        src: src, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy, theta: tB,
                        cx: bCx, cy: bCy, f: focal, rad: radius,
                        mirror: true
                    )
                    r = aF * fr + (1 - aF) * br2
                    g = aF * fg + (1 - aF) * bg2
                    b = aF * fb + (1 - aF) * bb2
                }

                out[idx]     = UInt8(clamping: Int(r))
                out[idx + 1] = UInt8(clamping: Int(g))
                out[idx + 2] = UInt8(clamping: Int(b))
                out[idx + 3] = 255
            }
        }

        guard let outCtx = CGContext(
            data: &out, width: outW, height: outH,
            bitsPerComponent: 8, bytesPerRow: outW * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ), let result = outCtx.makeImage() else { return nil }
        return UIImage(cgImage: result)
    }

    private static func sampleLens(
        src: [UInt8], stride: Int, srcW: Int, srcH: Int,
        bx: Float, by: Float, rxy: Float, theta: Float,
        cx: Float, cy: Float, f: Float, rad: Float,
        mirror: Bool
    ) -> (Float, Float, Float) {
        let r = f * theta
        guard r < rad else { return (0, 0, 0) }

        let px: Float, py: Float
        if rxy > 1e-6 {
            let dx = mirror ? -bx : bx
            px = cx + r * (dx / rxy)
            py = cy + r * (by / rxy)
        } else {
            px = cx; py = cy
        }

        let ix = Int(px), iy = Int(py)
        guard ix >= 0 && ix < srcW - 1 && iy >= 0 && iy < srcH - 1 else {
            return (0, 0, 0)
        }

        let fx = px - Float(ix), fy = py - Float(iy)
        let w00 = (1 - fx) * (1 - fy)
        let w10 = fx * (1 - fy)
        let w01 = (1 - fx) * fy
        let w11 = fx * fy

        let base = iy * stride + ix * 4
        var rv: Float = 0, gv: Float = 0, bv: Float = 0
        for (off, wt) in [(base, w00),
                           (base + 4, w10),
                           (base + stride, w01),
                           (base + stride + 4, w11)] {
            rv += Float(src[off]) * wt
            gv += Float(src[off + 1]) * wt
            bv += Float(src[off + 2]) * wt
        }
        return (rv, gv, bv)
    }
}
