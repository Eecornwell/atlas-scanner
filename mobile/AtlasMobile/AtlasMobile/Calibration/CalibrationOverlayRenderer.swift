import UIKit
import simd

/// Renders a visual calibration verification overlay.
/// Left panel: red=Insta360 edges, green=LiDAR projection, yellow=aligned.
/// Right panel: depth-coloured LiDAR dots on Insta360 panorama.
/// Mirrors atlas verify_seed_overlay.py seed_composite.jpg output.
enum CalibrationOverlayRenderer {

    static func render(frame: CalibrationFrame, extrinsic: RigidTransform) -> UIImage? {
        guard let erpCG = frame.erpImage.cgImage else { return nil }
        let erpW = erpCG.width, erpH = erpCG.height

        let lidarPixels = LiDARERPRenderer.project(
            frame: frame, extrinsic: extrinsic, erpW: erpW, erpH: erpH
        )

        // Render Insta360 ERP as RGBA
        guard let erpRGBA = renderRGBA(cgImage: erpCG) else { return nil }

        // Build edge maps
        let camEdges = sobelEdges(gray: grayPixels(from: erpCG), w: erpW, h: erpH)
        let lidEdges = sobelEdges(gray: lidarPixels, w: erpW, h: erpH)

        // Left panel: edge alignment
        var edgePanel = erpRGBA.map { px -> RGBA in
            RGBA(r: UInt8(Float(px.r) * 0.3),
                 g: UInt8(Float(px.g) * 0.3),
                 b: UInt8(Float(px.b) * 0.3), a: 255)
        }
        for i in 0..<(erpW * erpH) {
            let isCam = camEdges[i] > 0.3
            let isLid = lidEdges[i] > 0.3
            if isCam && isLid { edgePanel[i] = RGBA(r: 0, g: 220, b: 220, a: 255) }  // yellow
            else if isCam     { edgePanel[i] = RGBA(r: 220, g: 0, b: 0, a: 255) }    // red
            else if isLid     { edgePanel[i] = RGBA(r: 0, g: 220, b: 0, a: 255) }    // green
        }

        // Right panel: depth dots on ERP
        var overlayPanel = erpRGBA
        for i in 0..<(erpW * erpH) {
            let d = lidarPixels[i]
            guard d > 10 else { continue }
            let t = Float(d) / 255.0
            // TURBO-like: near=white, far=blue
            let r = UInt8(max(0, min(255, 255 * (1 - t))))
            let g = UInt8(max(0, min(255, 255 * (1 - abs(2*t - 1)))))
            let b = UInt8(max(0, min(255, 255 * t)))
            overlayPanel[i] = RGBA(r: r, g: g, b: b, a: 255)
        }

        // Composite side by side at half width each
        let halfW = erpW / 2
        return compositeHStack(
            left: edgePanel, right: overlayPanel,
            srcW: erpW, srcH: erpH, outW: halfW
        )
    }

    // MARK: - Image helpers

    private struct RGBA { var r, g, b, a: UInt8 }

    private static func grayPixels(from cgImage: CGImage) -> [UInt8] {
        let w = cgImage.width, h = cgImage.height
        var buf = [UInt8](repeating: 0, count: w * h)
        guard let ctx = CGContext(data: &buf, width: w, height: h,
                                  bitsPerComponent: 8, bytesPerRow: w,
                                  space: CGColorSpaceCreateDeviceGray(),
                                  bitmapInfo: CGImageAlphaInfo.none.rawValue) else { return buf }
        ctx.draw(cgImage, in: CGRect(x: 0, y: 0, width: w, height: h))
        return buf
    }

    private static func renderRGBA(cgImage: CGImage) -> [RGBA]? {
        let w = cgImage.width, h = cgImage.height
        var buf = [UInt8](repeating: 0, count: w * h * 4)
        guard let ctx = CGContext(data: &buf, width: w, height: h,
                                  bitsPerComponent: 8, bytesPerRow: w * 4,
                                  space: CGColorSpaceCreateDeviceRGB(),
                                  bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue) else { return nil }
        ctx.draw(cgImage, in: CGRect(x: 0, y: 0, width: w, height: h))
        return stride(from: 0, to: buf.count, by: 4).map {
            RGBA(r: buf[$0], g: buf[$0+1], b: buf[$0+2], a: buf[$0+3])
        }
    }

    private static func sobelEdges(gray: [UInt8], w: Int, h: Int) -> [Float] {
        var out = [Float](repeating: 0, count: w * h)
        for y in 1..<(h-1) {
            for x in 1..<(w-1) {
                let gx = -Int(gray[(y-1)*w+(x-1)]) - 2*Int(gray[y*w+(x-1)]) - Int(gray[(y+1)*w+(x-1)])
                       + Int(gray[(y-1)*w+(x+1)]) + 2*Int(gray[y*w+(x+1)]) + Int(gray[(y+1)*w+(x+1)])
                let gy = -Int(gray[(y-1)*w+(x-1)]) - 2*Int(gray[(y-1)*w+x]) - Int(gray[(y-1)*w+(x+1)])
                       + Int(gray[(y+1)*w+(x-1)]) + 2*Int(gray[(y+1)*w+x]) + Int(gray[(y+1)*w+(x+1)])
                out[y*w+x] = Float(gx*gx + gy*gy)
            }
        }
        let maxV = out.max() ?? 1
        if maxV > 0 { for i in 0..<out.count { out[i] /= maxV } }
        return out
    }

    private static func compositeHStack(
        left: [RGBA], right: [RGBA],
        srcW: Int, srcH: Int, outW: Int
    ) -> UIImage? {
        let totalW = outW * 2 + 4  // 4px divider
        var pixels = [UInt8](repeating: 0, count: totalW * srcH * 4)

        func writePanel(_ panel: [RGBA], offsetX: Int) {
            for y in 0..<srcH {
                for x in 0..<outW {
                    let srcX = x * srcW / outW
                    let src = panel[y * srcW + srcX]
                    let dst = (y * totalW + offsetX + x) * 4
                    pixels[dst] = src.r; pixels[dst+1] = src.g
                    pixels[dst+2] = src.b; pixels[dst+3] = src.a
                }
            }
        }
        writePanel(left, offsetX: 0)
        // Cyan divider
        for y in 0..<srcH {
            for x in outW..<(outW+4) {
                let dst = (y * totalW + x) * 4
                pixels[dst] = 0; pixels[dst+1] = 255; pixels[dst+2] = 255; pixels[dst+3] = 255
            }
        }
        writePanel(right, offsetX: outW + 4)

        let provider = CGDataProvider(data: Data(pixels) as CFData)!
        guard let cg = CGImage(width: totalW, height: srcH,
                               bitsPerComponent: 8, bitsPerPixel: 32,
                               bytesPerRow: totalW * 4,
                               space: CGColorSpaceCreateDeviceRGB(),
                               bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue),
                               provider: provider,
                               decode: nil, shouldInterpolate: false,
                               intent: .defaultIntent) else { return nil }
        return UIImage(cgImage: cg)
    }
}
