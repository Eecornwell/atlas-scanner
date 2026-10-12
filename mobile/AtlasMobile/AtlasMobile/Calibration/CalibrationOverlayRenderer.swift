import UIKit
import simd

/// Renders a perspective verification overlay for extrinsic calibration,
/// matching the visual style of tune_calibration.py --verify.
///
/// Single perspective crop of the Insta360 ERP (using the iPhone's actual
/// intrinsics) with JET-colored LiDAR depth dots blended at 35/65.
/// Rotated 90° CW for portrait display.
enum CalibrationOverlayRenderer {

    static func render(
        frame: CalibrationFrame,
        extrinsic: RigidTransform
    ) -> UIImage? {
        guard let erp = normalizedERPPixels(frame.erpImage) else {
            return nil
        }
        let erpW = erp.w, erpH = erp.h, erpRGBA = erp.pixels

        let T = ExtrinsicOptimizer.seedToCameraMatrix(extrinsic)
        let R = simd_float3x3(
            SIMD3(T.columns.0.x, T.columns.0.y, T.columns.0.z),
            SIMD3(T.columns.1.x, T.columns.1.y, T.columns.1.z),
            SIMD3(T.columns.2.x, T.columns.2.y, T.columns.2.z)
        )
        let t = SIMD3<Float>(
            T.columns.3.x, T.columns.3.y, T.columns.3.z
        )

        let outScale: Float = 3.0
        let di = frame.depthIntrinsics()
        let outW = Int(Float(frame.depthW) * outScale)
        let outH = Int(Float(frame.depthH) * outScale)
        let fxV = di.fx * outScale
        let fyV = di.fy * outScale
        let cxV = di.cx * outScale
        let cyV = di.cy * outScale
        let wF = Float(erpW), hF = Float(erpH)

        // Seam columns to dim in the perspective view
        let seamCols = ExtrinsicOptimizer.seamColumns(erpW: erpW)

        // Sample ERP background through iPhone's perspective
        var pixels = [RGBA](
            repeating: RGBA(r: 0, g: 0, b: 0, a: 255),
            count: outW * outH
        )
        for py in 0..<outH {
            for px in 0..<outW {
                let rxV = (Float(px) - cxV) / fxV
                let ryV = (Float(py) - cyV) / fyV
                let ray = simd_normalize(
                    SIMD3<Float>(-rxV, -ryV, 1)
                )
                let b = ExtrinsicOptimizer.erpCorrection * (R * ray)
                let lat = -asin(max(-1, min(1, b.y)))
                let lon = atan2(b.x, b.z)
                var u = Int(wF * (0.5 + lon / (2 * .pi)))
                u = ((u % erpW) + erpW) % erpW
                let v = max(0, min(erpH - 1,
                    Int(hF * (0.5 - lat / .pi))))
                var px4 = erpRGBA[v * erpW + u]
                if seamCols.contains(u) {
                    px4.r = UInt8(Float(px4.r) * 0.3)
                    px4.g = UInt8(Float(px4.g) * 0.3)
                    px4.b = UInt8(min(255,
                        Float(px4.b) * 0.3 + 50))
                }
                pixels[py * outW + px] = px4
            }
        }

        // Unproject LiDAR depth to ERP bearings, then project back
        // through the iPhone's perspective to find overlay positions.
        // Dots land where LiDAR maps in the ERP view — misalignment
        // means the extrinsic is off.
        struct Dot { let px: Int; let py: Int; let dist: Float }
        var dots = [Dot]()
        dots.reserveCapacity(frame.depthW * frame.depthH / 4)
        let Rt = R.transpose
        let erpCt = ExtrinsicOptimizer.erpCorrection.transpose

        for row in 0..<frame.depthH {
            for col in 0..<frame.depthW {
                let z = frame.depth[row * frame.depthW + col]
                guard z > 0.15 && z < 5.0 else { continue }
                let x = (Float(col) - di.cx) / di.fx * z
                let y = (Float(row) - di.cy) / di.fy * z
                let pt = R * SIMD3(-x, -y, z) + t
                let norm = simd_length(pt)
                guard norm > 0.3 else { continue }
                let b = ExtrinsicOptimizer.erpCorrection
                    * (pt / norm)

                // Inverse-project bearing to perspective pixel
                let cam = Rt * (erpCt * b)
                guard cam.z > 0 else { continue }
                let dpx = Int(-cam.x / cam.z * fxV + cxV)
                let dpy = Int(-cam.y / cam.z * fyV + cyV)
                guard dpx >= 3, dpx < outW - 3,
                      dpy >= 3, dpy < outH - 3
                else { continue }
                dots.append(Dot(px: dpx, py: dpy, dist: norm))
            }
        }

        guard !dots.isEmpty else {
            return toImage(pixels, w: outW, h: outH)
        }

        let dMin = dots.min(by: { $0.dist < $1.dist })!.dist
        let dMax = dots.max(by: { $0.dist < $1.dist })!.dist
        let dRange = dMax - dMin + 1e-6
        let circleR = 3
        let r2 = circleR * circleR

        for dot in dots {
            let tNorm = (dot.dist - dMin) / dRange
            let (jr, jg, jb) = jet(tNorm)
            for dy in -circleR...circleR {
                for dx in -circleR...circleR {
                    guard dx * dx + dy * dy <= r2 else {
                        continue
                    }
                    let idx = (dot.py + dy) * outW + (dot.px + dx)
                    let bg = pixels[idx]
                    pixels[idx] = RGBA(
                        r: clampByte(
                            Float(bg.r) * 0.65 + jr * 0.35
                        ),
                        g: clampByte(
                            Float(bg.g) * 0.65 + jg * 0.35
                        ),
                        b: clampByte(
                            Float(bg.b) * 0.65 + jb * 0.35
                        ),
                        a: 255
                    )
                }
            }
        }

        return toImage(pixels, w: outW, h: outH)
    }

    // MARK: - JET colormap (matches cv2.COLORMAP_JET)

    private static func jet(_ t: Float) -> (Float, Float, Float) {
        let c = max(0, min(1, t))
        let r: Float, g: Float, b: Float
        if c < 0.125 {
            r = 0; g = 0
            b = 128 + c / 0.125 * 127
        } else if c < 0.375 {
            r = 0
            g = (c - 0.125) / 0.25 * 255
            b = 255
        } else if c < 0.625 {
            r = (c - 0.375) / 0.25 * 255
            g = 255
            b = 255 - (c - 0.375) / 0.25 * 255
        } else if c < 0.875 {
            r = 255
            g = 255 - (c - 0.625) / 0.25 * 255
            b = 0
        } else {
            r = 255 - (c - 0.875) / 0.125 * 127
            g = 0; b = 0
        }
        return (r, g, b)
    }

    // MARK: - ERP image normalization

    private static func normalizedERPPixels(
        _ image: UIImage
    ) -> (pixels: [RGBA], w: Int, h: Int)? {
        let w = Int(image.size.width)
        let h = Int(image.size.height)
        guard w > 0, h > 0 else { return nil }

        if image.imageOrientation == .up, let cg = image.cgImage,
           cg.width == w, cg.height == h {
            guard let px = renderRGBA(cgImage: cg) else { return nil }
            return (px, w, h)
        }

        let renderer = UIGraphicsImageRenderer(
            size: CGSize(width: w, height: h)
        )
        let normalized = renderer.image { _ in
            image.draw(in: CGRect(x: 0, y: 0, width: w, height: h))
        }
        guard let cg = normalized.cgImage else { return nil }
        guard let px = renderRGBA(cgImage: cg) else { return nil }
        return (px, cg.width, cg.height)
    }

    // MARK: - Image helpers

    private struct RGBA { var r, g, b, a: UInt8 }

    private static func clampByte(_ v: Float) -> UInt8 {
        UInt8(max(0, min(255, Int(v))))
    }

    private static func renderRGBA(cgImage: CGImage) -> [RGBA]? {
        let w = cgImage.width, h = cgImage.height
        var buf = [UInt8](repeating: 0, count: w * h * 4)
        guard let ctx = CGContext(
            data: &buf, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return nil }
        ctx.draw(cgImage, in: CGRect(x: 0, y: 0, width: w, height: h))
        return Swift.stride(from: 0, to: buf.count, by: 4).map {
            RGBA(
                r: buf[$0], g: buf[$0+1],
                b: buf[$0+2], a: buf[$0+3]
            )
        }
    }

    private static func toImage(
        _ pixels: [RGBA], w: Int, h: Int
    ) -> UIImage? {
        var raw = [UInt8](repeating: 0, count: w * h * 4)
        for i in 0..<(w * h) {
            raw[i * 4] = pixels[i].r
            raw[i * 4 + 1] = pixels[i].g
            raw[i * 4 + 2] = pixels[i].b
            raw[i * 4 + 3] = 255
        }
        guard let provider = CGDataProvider(
            data: Data(raw) as CFData
        ) else { return nil }
        guard let cg = CGImage(
            width: w, height: h,
            bitsPerComponent: 8, bitsPerPixel: 32,
            bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGBitmapInfo(
                rawValue: CGImageAlphaInfo.premultipliedLast.rawValue
            ),
            provider: provider,
            decode: nil, shouldInterpolate: false,
            intent: .defaultIntent
        ) else { return nil }
        return UIImage(
            cgImage: cg, scale: 1.0, orientation: .right
        )
    }
}
