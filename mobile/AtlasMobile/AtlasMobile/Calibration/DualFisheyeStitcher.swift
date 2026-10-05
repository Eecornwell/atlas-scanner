import UIKit

/// Converts Insta360 ONE X2 dual-fisheye images to equirectangular (ERP).
///
/// The ONE X2 stores photos as two side-by-side equidistant fisheye circles
/// (front lens on the left, back lens on the right). This stitcher reprojects
/// them into a standard lat/lon ERP image that the rest of the calibration
/// and export pipeline expects.
enum DualFisheyeStitcher {

    /// Returns true if the image appears to be dual-fisheye rather than ERP.
    /// Checks for ~2:1 aspect ratio with dark (black) corners.
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

    /// Stitch a dual-fisheye image to equirectangular.
    ///
    /// - Parameters:
    ///   - dualFisheye: The raw dual-fisheye image from the Insta360 ONE X2.
    ///   - outputWidth: Width of the output ERP (height = width / 2).
    /// - Returns: An equirectangular UIImage, or nil on failure.
    static func stitch(
        _ dualFisheye: UIImage,
        outputWidth: Int = 2048
    ) -> UIImage? {
        guard let cg = dualFisheye.cgImage else { return nil }
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

        // Equidistant fisheye model for ONE X2 (~200° per lens)
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
                        src: src, stride: srcStride, srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy, theta: tF,
                        cx: fCx, cy: fCy, f: focal, rad: radius,
                        mirror: false
                    )
                } else if tB < blendLo {
                    (r, g, b) = sampleLens(
                        src: src, stride: srcStride, srcW: srcW, srcH: srcH,
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
                        src: src, stride: srcStride, srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy, theta: tF,
                        cx: fCx, cy: fCy, f: focal, rad: radius,
                        mirror: false
                    )
                    let (br2, bg2, bb2) = sampleLens(
                        src: src, stride: srcStride, srcW: srcW, srcH: srcH,
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

    // MARK: - Private

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
