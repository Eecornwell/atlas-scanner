import UIKit
import simd

enum CalibrationOverlayRenderer {

    struct Pair {
        let iphone: UIImage?
        let erpSlice: UIImage
        let depthOnErp: UIImage?
    }

    static func render(
        frame: CalibrationFrame,
        extrinsic: RigidTransform
    ) -> Pair? {
        guard let erp = normalizedERPPixels(
            frame.erpImage
        ) else { return nil }
        let erpW = erp.w, erpH = erp.h
        let erpRGBA = erp.pixels

        let T = ExtrinsicOptimizer.seedToCameraMatrix(
            extrinsic
        )
        let R = simd_float3x3(
            SIMD3(
                T.columns.0.x, T.columns.0.y,
                T.columns.0.z
            ),
            SIMD3(
                T.columns.1.x, T.columns.1.y,
                T.columns.1.z
            ),
            SIMD3(
                T.columns.2.x, T.columns.2.y,
                T.columns.2.z
            )
        )

        let outScale: Float = 3.0
        let di = frame.depthIntrinsics()
        let lsW = Int(Float(frame.depthW) * outScale)
        let lsH = Int(Float(frame.depthH) * outScale)
        let fxV = di.fx * outScale
        let fyV = di.fy * outScale
        let cxV = di.cx * outScale
        let cyV = di.cy * outScale
        let wF = Float(erpW), hF = Float(erpH)

        // Portrait output via CCW 90° rotation.
        // For portrait pixel (sx, sy):
        //   landscape col = lsW - 1 - sy
        //   landscape row = sx
        let pW = lsH, pH = lsW
        var erpPixels = [RGBA](
            repeating: RGBA(r: 0, g: 0, b: 0, a: 255),
            count: pW * pH
        )

        for sy in 0..<pH {
            for sx in 0..<pW {
                let px = lsW - 1 - sy
                let py = sx
                let rxV = (Float(px) - cxV) / fxV
                let ryV = (Float(py) - cyV) / fyV
                let ray = simd_normalize(
                    SIMD3<Float>(-rxV, -ryV, 1)
                )
                let b = R * ray
                let lat = -asin(max(-1, min(1, b.y)))
                let lon = atan2(b.x, b.z)
                var u = Int(
                    wF * (0.5 + lon / (2 * .pi))
                )
                u = ((u % erpW) + erpW) % erpW
                let v = max(0, min(erpH - 1,
                    Int(hF * (0.5 - lat / .pi))))
                erpPixels[sy * pW + sx] =
                    erpRGBA[v * erpW + u]
            }
        }

        guard let erpImage = toImage(
            erpPixels, w: pW, h: pH,
            orientation: .up
        ) else { return nil }

        let iphoneImage = rotateToPortrait(
            frame.iphoneImage, lsW: lsW, lsH: lsH,
            portraitW: pW, portraitH: pH
        )

        let depthImage = renderDepthMap(
            frame.depth,
            depthW: frame.depthW,
            depthH: frame.depthH,
            landscapeW: lsW, landscapeH: lsH,
            portraitW: pW, portraitH: pH
        )

        var depthOnErp: UIImage?
        if let depth = depthImage {
            let size = erpImage.size
            let renderer = UIGraphicsImageRenderer(
                size: size
            )
            depthOnErp = renderer.image { _ in
                let rect = CGRect(
                    origin: .zero, size: size
                )
                erpImage.draw(in: rect)
                depth.draw(
                    in: rect,
                    blendMode: .normal,
                    alpha: 0.4
                )
            }
        }

        return Pair(
            iphone: iphoneImage,
            erpSlice: erpImage,
            depthOnErp: depthOnErp
        )
    }

    // MARK: - Depth colormap

    private static func renderDepthMap(
        _ depth: [Float],
        depthW: Int, depthH: Int,
        landscapeW lsW: Int, landscapeH lsH: Int,
        portraitW pW: Int, portraitH pH: Int
    ) -> UIImage? {
        guard !depth.isEmpty else { return nil }

        var minD: Float = .greatestFiniteMagnitude
        var maxD: Float = 0
        for d in depth where d > 0 && d.isFinite {
            if d < minD { minD = d }
            if d > maxD { maxD = d }
        }
        guard maxD > minD else { return nil }
        let range = maxD - minD

        let scaleX = Float(depthW) / Float(lsW)
        let scaleY = Float(depthH) / Float(lsH)

        var pixels = [RGBA](
            repeating: RGBA(r: 0, g: 0, b: 0, a: 255),
            count: pW * pH
        )

        for sy in 0..<pH {
            for sx in 0..<pW {
                let px = sy
                let py = pW - 1 - sx
                let dsx = min(
                    Int(Float(px) * scaleX),
                    depthW - 1
                )
                let dsy = min(
                    Int(Float(py) * scaleY),
                    depthH - 1
                )
                let d = depth[dsy * depthW + dsx]
                let idx = sy * pW + sx
                guard d > 0, d.isFinite else {
                    pixels[idx] = RGBA(
                        r: 0, g: 0, b: 0, a: 255
                    )
                    continue
                }
                let t = (d - minD) / range
                pixels[idx] = turboColor(t)
            }
        }

        return toImage(
            pixels, w: pW, h: pH,
            orientation: .up
        )
    }

    private static func turboColor(
        _ t: Float
    ) -> RGBA {
        let t = max(0, min(1, t))
        let r: Float
        let g: Float
        let b: Float
        if t < 0.25 {
            let s = t / 0.25
            r = 0.18 + 0.50 * s
            g = 0.0 + 0.54 * s
            b = 0.53 + 0.47 * s
        } else if t < 0.5 {
            let s = (t - 0.25) / 0.25
            r = 0.68 + 0.24 * s
            g = 0.54 + 0.37 * s
            b = 1.0 - 0.65 * s
        } else if t < 0.75 {
            let s = (t - 0.5) / 0.25
            r = 0.92 + 0.08 * s
            g = 0.91 - 0.23 * s
            b = 0.35 - 0.25 * s
        } else {
            let s = (t - 0.75) / 0.25
            r = 1.0 - 0.45 * s
            g = 0.68 - 0.52 * s
            b = 0.10 + 0.05 * s
        }
        return RGBA(
            r: UInt8(r * 255),
            g: UInt8(g * 255),
            b: UInt8(b * 255),
            a: 255
        )
    }

    // MARK: - iPhone image rotation

    private static func rotateToPortrait(
        _ source: UIImage?,
        lsW: Int, lsH: Int,
        portraitW pW: Int, portraitH pH: Int
    ) -> UIImage? {
        guard let source,
              let srcCG = source.cgImage
        else { return nil }

        guard let srcPixels = renderRGBA(
            cgImage: srcCG, targetW: lsW, targetH: lsH
        ) else { return nil }

        var pixels = [RGBA](
            repeating: RGBA(r: 0, g: 0, b: 0, a: 255),
            count: pW * pH
        )
        // iPhone pixels use CW 90° (CIImage→CGImage
        // has opposite Y from the depth buffer).
        for sy in 0..<pH {
            for sx in 0..<pW {
                let px = sy
                let py = pW - 1 - sx
                pixels[sy * pW + sx] =
                    srcPixels[py * lsW + px]
            }
        }
        return toImage(
            pixels, w: pW, h: pH,
            orientation: .up
        )
    }

    // MARK: - ERP image normalization

    private static func normalizedERPPixels(
        _ image: UIImage
    ) -> (pixels: [RGBA], w: Int, h: Int)? {
        let w = Int(image.size.width)
        let h = Int(image.size.height)
        guard w > 0, h > 0 else { return nil }

        if image.imageOrientation == .up,
           let cg = image.cgImage,
           cg.width == w, cg.height == h {
            guard let px = renderRGBA(cgImage: cg)
            else { return nil }
            return (px, w, h)
        }

        let renderer = UIGraphicsImageRenderer(
            size: CGSize(width: w, height: h)
        )
        let normalized = renderer.image { _ in
            image.draw(
                in: CGRect(
                    x: 0, y: 0, width: w, height: h
                )
            )
        }
        guard let cg = normalized.cgImage else {
            return nil
        }
        guard let px = renderRGBA(cgImage: cg) else {
            return nil
        }
        return (px, cg.width, cg.height)
    }

    // MARK: - Image helpers

    private struct RGBA { var r, g, b, a: UInt8 }

    private static func renderRGBA(
        cgImage: CGImage
    ) -> [RGBA]? {
        renderRGBA(
            cgImage: cgImage,
            targetW: cgImage.width,
            targetH: cgImage.height
        )
    }

    private static func renderRGBA(
        cgImage: CGImage,
        targetW w: Int, targetH h: Int
    ) -> [RGBA]? {
        var buf = [UInt8](
            repeating: 0, count: w * h * 4
        )
        guard let ctx = CGContext(
            data: &buf, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo
                .premultipliedLast.rawValue
        ) else { return nil }
        ctx.draw(
            cgImage,
            in: CGRect(
                x: 0, y: 0, width: w, height: h
            )
        )
        return Swift.stride(
            from: 0, to: buf.count, by: 4
        ).map {
            RGBA(
                r: buf[$0], g: buf[$0 + 1],
                b: buf[$0 + 2], a: buf[$0 + 3]
            )
        }
    }

    private static func toImage(
        _ pixels: [RGBA], w: Int, h: Int,
        orientation: UIImage.Orientation = .up
    ) -> UIImage? {
        var raw = [UInt8](
            repeating: 0, count: w * h * 4
        )
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
            bitsPerComponent: 8,
            bitsPerPixel: 32,
            bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGBitmapInfo(
                rawValue: CGImageAlphaInfo
                    .premultipliedLast.rawValue
            ),
            provider: provider,
            decode: nil,
            shouldInterpolate: false,
            intent: .defaultIntent
        ) else { return nil }
        return UIImage(
            cgImage: cg,
            scale: 1.0,
            orientation: orientation
        )
    }
}
