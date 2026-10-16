import UIKit
import simd

/// Renders a verification overlay for extrinsic calibration.
///
/// Blends the iPhone RGB with the Insta360 ERP reprojected to the
/// iPhone's perspective (50/50). If the extrinsic is correct, edges
/// from both cameras align cleanly. If it's off, edges look doubled.
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

        let outScale: Float = 3.0
        let di = frame.depthIntrinsics()
        let outW = Int(Float(frame.depthW) * outScale)
        let outH = Int(Float(frame.depthH) * outScale)
        let fxV = di.fx * outScale
        let fyV = di.fy * outScale
        let cxV = di.cx * outScale
        let cyV = di.cy * outScale
        let wF = Float(erpW), hF = Float(erpH)

        // Get iPhone RGB pixels scaled to output size
        let iphonePixels = frame.iphoneImage.flatMap {
            scaledRGBA($0, toWidth: outW, height: outH)
        }

        var pixels = [RGBA](
            repeating: RGBA(r: 0, g: 0, b: 0, a: 255),
            count: outW * outH
        )

        for py in 0..<outH {
            for px in 0..<outW {
                // Reproject ERP to iPhone perspective
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
                let erpPx = erpRGBA[v * erpW + u]

                let idx = py * outW + px
                if let iphone = iphonePixels {
                    let iPx = iphone[idx]
                    pixels[idx] = RGBA(
                        r: UInt8((Int(iPx.r) + Int(erpPx.r)) / 2),
                        g: UInt8((Int(iPx.g) + Int(erpPx.g)) / 2),
                        b: UInt8((Int(iPx.b) + Int(erpPx.b)) / 2),
                        a: 255
                    )
                } else {
                    pixels[idx] = erpPx
                }
            }
        }

        return toImage(pixels, w: outW, h: outH)
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

    private static func scaledRGBA(
        _ image: UIImage, toWidth w: Int, height h: Int
    ) -> [RGBA]? {
        let renderer = UIGraphicsImageRenderer(
            size: CGSize(width: w, height: h)
        )
        let scaled = renderer.image { _ in
            image.draw(in: CGRect(x: 0, y: 0, width: w, height: h))
        }
        guard let cg = scaled.cgImage else { return nil }
        return renderRGBA(cgImage: cg)
    }

    // MARK: - Image helpers

    private struct RGBA { var r, g, b, a: UInt8 }

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
