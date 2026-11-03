import UIKit
import ImageIO
import simd

/// Converts Insta360 dual-fisheye images to equirectangular (ERP).
///
/// Insta360 cameras (ONE X2, X5, etc.) store photos as two side-by-side
/// equidistant fisheye circles (front lens left, back lens right). This
/// stitcher reprojects them into a standard lat/lon ERP image that the
/// rest of the calibration and export pipeline expects.
enum DualFisheyeStitcher {

    /// Roll correction for a given camera model's dual-fisheye output.
    static func rollCorrection(forModel model: String) -> Float {
        let m = model.lowercased()
        if m.contains("x2") || m.contains("one x2") {
            return -90
        }
        return 0
    }

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

    /// Stitch from a file URL — uses ImageIO to downsample during
    /// decode, avoiding a full-resolution pixel buffer in memory.
    static func stitch(
        url: URL,
        outputWidth: Int = 2048,
        rollDeg: Float = 0,
        allowSubsample: Bool = false
    ) -> UIImage? {
        guard let px = stitchToPixels(
            url: url, outputWidth: outputWidth,
            rollDeg: rollDeg,
            allowSubsample: allowSubsample
        ) else { return nil }
        return pixelsToImage(
            px.px, width: px.w, height: px.h
        )
    }

    /// Stitch and return raw RGBA pixels (no UIImage).
    /// Avoids the double-allocation of UIImage + pixel
    /// extraction when callers only need the buffer.
    static func stitchToPixels(
        url: URL,
        outputWidth: Int = 2048,
        rollDeg: Float = 0,
        allowSubsample: Bool = false
    ) -> (px: [UInt8], w: Int, h: Int)? {
        let srcMax = max(
            3072, outputWidth + outputWidth / 4
        )
        let src = autoreleasepool {
            () -> (d: [UInt8], w: Int, h: Int)? in
            guard let source =
                CGImageSourceCreateWithURL(
                    url as CFURL, nil
                ),
                let cg = downsampleSource(
                    source, maxWidth: srcMax,
                    subsample: allowSubsample
                )
            else { return nil }
            return extractPixels(cg)
        }
        guard let src else { return nil }
        return stitchPixelsToPixels(
            src.d, width: src.w, height: src.h,
            outputWidth: outputWidth,
            rollDeg: rollDeg
        )
    }

    static func stitch(
        _ dualFisheye: UIImage,
        outputWidth: Int = 2048,
        rollDeg: Float = 0
    ) -> UIImage? {
        guard let origCG = dualFisheye.cgImage
        else { return nil }
        let maxSrcW = max(
            3072, outputWidth + outputWidth / 4
        )
        let px = autoreleasepool {
            () -> (d: [UInt8], w: Int, h: Int)? in
            let cg: CGImage
            if origCG.width > maxSrcW {
                guard let data = dualFisheye.jpegData(
                    compressionQuality: 0.95
                ), let source =
                    CGImageSourceCreateWithData(
                        data as CFData, nil
                    ),
                    let ds = downsampleSource(
                        source, maxWidth: maxSrcW
                    )
                else { return nil }
                cg = ds
            } else {
                cg = origCG
            }
            return extractPixels(cg)
        }
        guard let px else { return nil }
        return stitchFromPixels(
            px.d, width: px.w, height: px.h,
            outputWidth: outputWidth,
            rollDeg: rollDeg
        )
    }

    private static func downsampleSource(
        _ source: CGImageSource,
        maxWidth: Int,
        subsample: Bool = false
    ) -> CGImage? {
        var options: [CFString: Any] = [
            kCGImageSourceThumbnailMaxPixelSize:
                maxWidth,
            kCGImageSourceCreateThumbnailFromImageAlways:
                true,
            kCGImageSourceCreateThumbnailWithTransform:
                true,
            kCGImageSourceShouldCacheImmediately:
                true,
            kCGImageSourceShouldCache:
                false,
        ]
        // Hint JPEG decoder to use 1/2 subsample for
        // large images (X5 72MP = 9504 wide), avoiding
        // a ~228 MB full-resolution decode buffer.
        if subsample,
            let props =
            CGImageSourceCopyPropertiesAtIndex(
                source, 0, nil
            ) as? [CFString: Any],
            let srcW = props[
                kCGImagePropertyPixelWidth
            ] as? Int,
            srcW > 8000
        {
            options[kCGImageSourceSubsampleFactor] = 2
        }
        return CGImageSourceCreateThumbnailAtIndex(
            source, 0, options as CFDictionary
        )
    }

    private static func extractPixels(
        _ cg: CGImage
    ) -> (d: [UInt8], w: Int, h: Int)? {
        let w = cg.width, h = cg.height
        guard w > 0, h > 0 else { return nil }
        var px = [UInt8](
            repeating: 0, count: w * h * 4
        )
        guard let ctx = CGContext(
            data: &px, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo
                .noneSkipLast.rawValue
        ) else { return nil }
        ctx.draw(
            cg,
            in: CGRect(x: 0, y: 0, width: w, height: h)
        )
        return (px, w, h)
    }

    static func pixelsToImage(
        _ px: [UInt8], width: Int, height: Int
    ) -> UIImage? {
        var buf = px
        guard let ctx = CGContext(
            data: &buf, width: width, height: height,
            bitsPerComponent: 8,
            bytesPerRow: width * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo
                .noneSkipLast.rawValue
        ), let cg = ctx.makeImage()
        else { return nil }
        return UIImage(cgImage: cg)
    }

    /// Load dual-fisheye pixels at reduced resolution via
    /// CGImageSource thumbnailing.
    static func loadPixels(
        url: URL, maxWidth: Int
    ) -> (px: [UInt8], w: Int, h: Int)? {
        guard let source = CGImageSourceCreateWithURL(
            url as CFURL, nil),
            let cg = downsampleSource(
                source, maxWidth: maxWidth,
                subsample: true)
        else { return nil }
        guard let r = extractPixels(cg) else { return nil }
        return (px: r.d, w: r.w, h: r.h)
    }

    private static func stitchFromPixels(
        _ src: [UInt8],
        width srcW: Int, height srcH: Int,
        outputWidth: Int,
        rollDeg: Float
    ) -> UIImage? {
        guard let px = stitchPixelsToPixels(
            src, width: srcW, height: srcH,
            outputWidth: outputWidth,
            rollDeg: rollDeg
        ) else { return nil }
        return pixelsToImage(
            px.px, width: px.w, height: px.h
        )
    }

    private static func stitchPixelsToPixels(
        _ src: [UInt8],
        width srcW: Int, height srcH: Int,
        outputWidth: Int,
        rollDeg: Float
    ) -> (px: [UInt8], w: Int, h: Int)? {
        let halfW = srcW / 2

        let radius = Float(srcH) * 0.47
        let halfFov: Float = 100 * .pi / 180
        let focal = radius / halfFov

        let fCx = Float(halfW) * 0.5
        let fCy = Float(srcH) * 0.5
        let bCx = Float(halfW) + Float(halfW) * 0.5
        let bCy = Float(srcH) * 0.5

        let outW = outputWidth
        let outH = outputWidth / 2
        var out = [UInt8](
            repeating: 0, count: outW * outH * 4
        )

        let W = Float(outW), H = Float(outH)
        let blendLo: Float = 85 * .pi / 180
        let blendHi: Float = 95 * .pi / 180
        let srcStride = srcW * 4

        let rollRad = rollDeg * .pi / 180
        let cosR = cos(rollRad), sinR = sin(rollRad)
        let applyRoll = abs(rollDeg) > 0.1

        for ov in 0..<outH {
            let lat = Float.pi * (0.5 - Float(ov) / H)
            let cosLat = cos(lat)
            let sinLat = sin(lat)

            for ou in 0..<outW {
                let lon = 2 * Float.pi
                    * (Float(ou) / W - 0.5)
                var bx = cosLat * sin(lon)
                var by = -sinLat
                let bz = cosLat * cos(lon)

                if applyRoll {
                    let bx0 = bx
                    bx = bx0 * cosR - by * sinR
                    by = bx0 * sinR + by * cosR
                }

                let rxy = sqrt(bx * bx + by * by)

                let tF = atan2(rxy, bz)
                let tB = atan2(rxy, -bz)

                let idx = (ov * outW + ou) * 4
                var r: Float, g: Float, b: Float

                if tF < blendLo {
                    (r, g, b) = sampleLens(
                        src: src, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tF,
                        cx: fCx, cy: fCy,
                        f: focal, rad: radius,
                        mirror: false
                    )
                } else if tB < blendLo {
                    (r, g, b) = sampleLens(
                        src: src, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tB,
                        cx: bCx, cy: bCy,
                        f: focal, rad: radius,
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
                        bx: bx, by: by, rxy: rxy,
                        theta: tF,
                        cx: fCx, cy: fCy,
                        f: focal, rad: radius,
                        mirror: false
                    )
                    let (br2, bg2, bb2) = sampleLens(
                        src: src, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tB,
                        cx: bCx, cy: bCy,
                        f: focal, rad: radius,
                        mirror: true
                    )
                    r = aF * fr + (1 - aF) * br2
                    g = aF * fg + (1 - aF) * bg2
                    b = aF * fb + (1 - aF) * bb2
                }

                out[idx] = UInt8(clamping: Int(r))
                out[idx + 1] = UInt8(
                    clamping: Int(g)
                )
                out[idx + 2] = UInt8(
                    clamping: Int(b)
                )
                out[idx + 3] = 255
            }
        }

        return (out, outW, outH)
    }

    // MARK: - Direct tile from dual-fisheye

    static func sliceTileFromDualFisheye(
        srcPx: [UInt8], srcW: Int, srcH: Int,
        tileSize: Int,
        faceRotation: simd_float3x3,
        tileFovDeg: Float,
        rollDeg: Float
    ) -> [UInt8]? {
        let halfW = srcW / 2
        let radius = Float(srcH) * 0.47
        let halfFov: Float = 100 * .pi / 180
        let focal = radius / halfFov

        let fCx = Float(halfW) * 0.5
        let fCy = Float(srcH) * 0.5
        let bCx = Float(halfW) + Float(halfW) * 0.5
        let bCy = Float(srcH) * 0.5

        let blendLo: Float = 85 * .pi / 180
        let blendHi: Float = 95 * .pi / 180
        let srcStride = srcW * 4

        let rollRad = rollDeg * .pi / 180
        let cosR = cos(rollRad), sinR = sin(rollRad)
        let applyRoll = abs(rollDeg) > 0.1

        let tileF = Float(tileSize)
            / (2.0 * tan(tileFovDeg * .pi / 360.0))
        let tileCtr = Float(tileSize) / 2.0

        let Rt = faceRotation.transpose
        let n = tileSize * tileSize
        var out = [UInt8](
            repeating: 0, count: n * 4)

        for row in 0..<tileSize {
            for col in 0..<tileSize {
                let rx = (Float(col) + 0.5 - tileCtr) / tileF
                let ry = (Float(row) + 0.5 - tileCtr) / tileF
                let rz: Float = 1.0
                let norm = sqrt(rx * rx + ry * ry + rz * rz)
                let ray = SIMD3<Float>(
                    rx / norm, ry / norm, rz / norm)
                let pano = Rt * ray

                var bx = pano.x
                var by = pano.y
                let bz = pano.z

                if applyRoll {
                    let bx0 = bx
                    bx = bx0 * cosR - by * sinR
                    by = bx0 * sinR + by * cosR
                }

                let rxy = sqrt(bx * bx + by * by)
                let tF = atan2(rxy, bz)
                let tB = atan2(rxy, -bz)

                let idx = (row * tileSize + col) * 4
                let r: Float, g: Float, b: Float

                if tF < blendLo {
                    (r, g, b) = sampleLens(
                        src: srcPx, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tF,
                        cx: fCx, cy: fCy,
                        f: focal, rad: radius,
                        mirror: false)
                } else if tB < blendLo {
                    (r, g, b) = sampleLens(
                        src: srcPx, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tB,
                        cx: bCx, cy: bCy,
                        f: focal, rad: radius,
                        mirror: true)
                } else {
                    let wF = max(0, blendHi - tF)
                    let wB = max(0, blendHi - tB)
                    let sum = wF + wB
                    let aF = sum > 0 ? wF / sum : 0.5
                    let (fr, fg, fb) = sampleLens(
                        src: srcPx, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tF,
                        cx: fCx, cy: fCy,
                        f: focal, rad: radius,
                        mirror: false)
                    let (br2, bg2, bb2) = sampleLens(
                        src: srcPx, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tB,
                        cx: bCx, cy: bCy,
                        f: focal, rad: radius,
                        mirror: true)
                    r = aF * fr + (1 - aF) * br2
                    g = aF * fg + (1 - aF) * bg2
                    b = aF * fb + (1 - aF) * bb2
                }

                out[idx] = UInt8(clamping: Int(r))
                out[idx + 1] = UInt8(clamping: Int(g))
                out[idx + 2] = UInt8(clamping: Int(b))
                out[idx + 3] = 255
            }
        }
        return out
    }

    /// Reproject dual-fisheye source directly to an equidistant
    /// fisheye image (front or back lens), skipping the ERP intermediate.
    /// Same sampleLens logic as sliceTileFromDualFisheye but with
    /// equidistant fisheye output grid instead of perspective tile grid.
    static func reprojectToFisheye(
        srcPx: [UInt8], srcW: Int, srcH: Int,
        fisheyeSize: Int,
        fisheyeHalfFovRad: Float,
        fisheyeRadiusFrac: Float,
        isFrontLens: Bool,
        rollDeg: Float
    ) -> [UInt8]? {
        let halfW = srcW / 2
        let srcRadius = Float(srcH) * 0.47
        let srcHalfFov: Float = 100 * .pi / 180
        let srcFocal = srcRadius / srcHalfFov

        let fCx = Float(halfW) * 0.5
        let fCy = Float(srcH) * 0.5
        let bCx = Float(halfW) + Float(halfW) * 0.5
        let bCy = Float(srcH) * 0.5

        let blendLo: Float = 85 * .pi / 180
        let blendHi: Float = 95 * .pi / 180
        let srcStride = srcW * 4

        let rollRad = rollDeg * .pi / 180
        let cosR = cos(rollRad), sinR = sin(rollRad)
        let applyRoll = abs(rollDeg) > 0.1

        let S = fisheyeSize
        let outRadius = Float(S) * fisheyeRadiusFrac
        let outFocal = outRadius / fisheyeHalfFovRad
        let cx = Float(S) / 2.0
        let cy = Float(S) / 2.0

        let zSign: Float = isFrontLens ? 1.0 : -1.0
        let xSign: Float = isFrontLens ? 1.0 : -1.0

        var out = [UInt8](repeating: 0, count: S * S * 4)

        for v in 0..<S {
            for u in 0..<S {
                let dx = Float(u) - cx
                let dy = Float(v) - cy
                let r = sqrtf(dx * dx + dy * dy)
                let idx = (v * S + u) * 4

                guard r < outRadius else { continue }

                if r < 0.001 {
                    let lCx = isFrontLens ? fCx : bCx
                    let lCy = isFrontLens ? fCy : bCy
                    let ix = Int(lCx), iy = Int(lCy)
                    guard ix >= 0, ix < srcW,
                          iy >= 0, iy < srcH
                    else { continue }
                    let off = iy * srcStride + ix * 4
                    out[idx] = srcPx[off]
                    out[idx + 1] = srcPx[off + 1]
                    out[idx + 2] = srcPx[off + 2]
                    out[idx + 3] = 255
                    continue
                }

                let theta = r / outFocal
                let sinT = sinf(theta)
                let cosT = cosf(theta)

                var bx = xSign * sinT * dx / r
                var by = sinT * dy / r
                let bz = zSign * cosT

                if applyRoll {
                    let bx0 = bx
                    bx = bx0 * cosR - by * sinR
                    by = bx0 * sinR + by * cosR
                }

                let rxy = sqrtf(bx * bx + by * by)
                let tF = atan2(rxy, bz)
                let tB = atan2(rxy, -bz)

                let rv: Float, gv: Float, bv: Float

                if tF < blendLo {
                    (rv, gv, bv) = sampleLens(
                        src: srcPx, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tF,
                        cx: fCx, cy: fCy,
                        f: srcFocal, rad: srcRadius,
                        mirror: false)
                } else if tB < blendLo {
                    (rv, gv, bv) = sampleLens(
                        src: srcPx, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tB,
                        cx: bCx, cy: bCy,
                        f: srcFocal, rad: srcRadius,
                        mirror: true)
                } else {
                    let wF = max(0, blendHi - tF)
                    let wB = max(0, blendHi - tB)
                    let sum = wF + wB
                    let aF = sum > 0 ? wF / sum : 0.5
                    let (fr, fg, fb) = sampleLens(
                        src: srcPx, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tF,
                        cx: fCx, cy: fCy,
                        f: srcFocal, rad: srcRadius,
                        mirror: false)
                    let (br2, bg2, bb2) = sampleLens(
                        src: srcPx, stride: srcStride,
                        srcW: srcW, srcH: srcH,
                        bx: bx, by: by, rxy: rxy,
                        theta: tB,
                        cx: bCx, cy: bCy,
                        f: srcFocal, rad: srcRadius,
                        mirror: true)
                    rv = aF * fr + (1 - aF) * br2
                    gv = aF * fg + (1 - aF) * bg2
                    bv = aF * fb + (1 - aF) * bb2
                }

                out[idx] = UInt8(clamping: Int(rv))
                out[idx + 1] = UInt8(clamping: Int(gv))
                out[idx + 2] = UInt8(clamping: Int(bv))
                out[idx + 3] = 255
            }
        }
        return out
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
