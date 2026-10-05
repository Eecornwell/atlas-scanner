import UIKit
import simd

/// Renders a perspective verification overlay for extrinsic calibration.
///
/// Both RGB (Insta360 ERP) and LiDAR are projected through the same ERP
/// intermediate and sampled at identical coordinates, guaranteeing
/// pixel-perfect alignment regardless of depth buffer orientation or
/// extrinsic rotation.
///
/// Left panel: edge alignment — red = camera edges, green = LiDAR edges,
///   yellow = aligned.
/// Right panel: TURBO-colored LiDAR dots on the Insta360 perspective crop.
enum CalibrationOverlayRenderer {

    static func render(
        frame: CalibrationFrame,
        extrinsic: RigidTransform
    ) -> UIImage? {
        // Normalize ERP orientation before extracting pixels.
        // UIImage may carry an EXIF rotation flag; cgImage ignores it,
        // which would cause us to sample a rotated pixel grid.
        let erpW: Int
        let erpH: Int
        let erpRGBA: [RGBA]
        if let result = normalizedERPPixels(frame.erpImage) {
            erpW = result.w
            erpH = result.h
            erpRGBA = result.pixels
        } else {
            return nil
        }

        // Project LiDAR depth into the same ERP space as the Insta360
        // image via LiDARERPRenderer. Both panels then sample from ERP
        // using identical ray casting, so alignment is guaranteed.
        let lidarERP = LiDARERPRenderer.project(
            frame: frame, extrinsic: extrinsic,
            erpW: erpW, erpH: erpH
        )

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

        // -- Perspective crop: sample both RGB and LiDAR from ERP --
        var perspRGBA = [RGBA](
            repeating: RGBA(r: 0, g: 0, b: 0, a: 255),
            count: outW * outH
        )
        var lidarRaw = [UInt8](repeating: 0, count: outW * outH)
        let wF = Float(erpW), hF = Float(erpH)

        for py in 0..<outH {
            for px in 0..<outW {
                let rxV = (Float(px) - cxV) / fxV
                let ryV = (Float(py) - cyV) / fyV
                let ray = simd_normalize(SIMD3<Float>(-rxV, -ryV, 1))
                let b = ExtrinsicOptimizer.erpCorrection * (R * ray)
                let lat = -asin(max(-1, min(1, b.y)))
                let lon = atan2(b.x, b.z)
                var u = Int(wF * (0.5 + lon / (2 * .pi)))
                u = ((u % erpW) + erpW) % erpW
                let v = max(0, min(erpH - 1,
                    Int(hF * (0.5 - lat / .pi))))

                let erpIdx = v * erpW + u
                perspRGBA[py * outW + px] = erpRGBA[erpIdx]
                lidarRaw[py * outW + px] = lidarERP[erpIdx]
            }
        }

        // Mask LiDAR where perspective crop is dark
        let camGray = perspRGBA.map {
            UInt8((Int($0.r) + Int($0.g) + Int($0.b)) / 3)
        }
        for i in 0..<(outW * outH) where camGray[i] < 30 {
            lidarRaw[i] = 0
        }
        let lidarMask = lidarRaw.map { $0 > 10 }

        // Dilate LiDAR for visibility and edge detection
        let lidarDilated = dilateGray(
            lidarRaw, w: outW, h: outH, radius: 2
        )

        // -- Left panel: edge alignment --
        let camEdges = sobelBinary(
            gray: camGray, w: outW, h: outH, thresh: 0.06
        )
        let lidEdges = sobelBinary(
            gray: lidarDilated, w: outW, h: outH, thresh: 0.04
        )

        var edgePanel = perspRGBA.map { px in
            RGBA(
                r: UInt8(Float(px.r) * 0.3),
                g: UInt8(Float(px.g) * 0.3),
                b: UInt8(Float(px.b) * 0.3), a: 255
            )
        }
        for i in 0..<(outW * outH) {
            let ce = camEdges[i], le = lidEdges[i]
            if ce && le {
                edgePanel[i] = RGBA(r: 220, g: 220, b: 0, a: 255)
            } else if ce {
                edgePanel[i] = RGBA(r: 220, g: 0, b: 0, a: 255)
            } else if le {
                edgePanel[i] = RGBA(r: 0, g: 220, b: 0, a: 255)
            }
        }

        // -- Right panel: TURBO-colored depth dots on perspective crop --
        var depthPanel = perspRGBA
        for i in 0..<(outW * outH) where lidarMask[i] {
            let t = Float(lidarDilated[i]) / 255.0
            let (tr, tg, tb) = turbo(t)
            depthPanel[i] = RGBA(
                r: clampByte(Float(perspRGBA[i].r) * 0.15 + tr * 0.85),
                g: clampByte(Float(perspRGBA[i].g) * 0.15 + tg * 0.85),
                b: clampByte(Float(perspRGBA[i].b) * 0.15 + tb * 0.85),
                a: 255
            )
        }

        // White halo around LiDAR dots
        let halo = haloFromMask(lidarMask, w: outW, h: outH)
        for i in 0..<(outW * outH) where halo[i] {
            depthPanel[i] = RGBA(r: 255, g: 255, b: 255, a: 255)
        }

        return compositeHStack(
            left: edgePanel, right: depthPanel,
            w: outW, h: outH
        )
    }

    // MARK: - ERP image normalization

    /// Draws the UIImage through UIGraphicsImageRenderer so EXIF
    /// orientation is baked into the pixel data, then extracts RGBA.
    private static func normalizedERPPixels(
        _ image: UIImage
    ) -> (pixels: [RGBA], w: Int, h: Int)? {
        let w = Int(image.size.width)
        let h = Int(image.size.height)
        guard w > 0, h > 0 else { return nil }

        // Fast path: no rotation needed
        if image.imageOrientation == .up, let cg = image.cgImage,
           cg.width == w, cg.height == h {
            guard let px = renderRGBA(cgImage: cg) else { return nil }
            return (px, w, h)
        }

        // Slow path: re-render to apply orientation
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

    // MARK: - Morphological operations

    private static func dilateGray(
        _ gray: [UInt8], w: Int, h: Int, radius: Int
    ) -> [UInt8] {
        var out = gray
        let r2 = radius * radius + 1
        for y in 0..<h {
            for x in 0..<w {
                let val = gray[y * w + x]
                guard val > 0 else { continue }
                for dy in -radius...radius {
                    let ny = y + dy
                    guard ny >= 0 && ny < h else { continue }
                    for dx in -radius...radius {
                        guard dx * dx + dy * dy <= r2 else { continue }
                        let nx = max(0, min(w - 1, x + dx))
                        let idx = ny * w + nx
                        if val > out[idx] { out[idx] = val }
                    }
                }
            }
        }
        return out
    }

    private static func haloFromMask(
        _ mask: [Bool], w: Int, h: Int
    ) -> [Bool] {
        var halo = [Bool](repeating: false, count: w * h)
        for y in 0..<h {
            for x in 0..<w {
                guard mask[y * w + x] else { continue }
                for dy in -3...3 {
                    let ny = y + dy
                    guard ny >= 0 && ny < h else { continue }
                    for dx in -3...3 {
                        let nx = max(0, min(w - 1, x + dx))
                        let idx = ny * w + nx
                        if !mask[idx] { halo[idx] = true }
                    }
                }
            }
        }
        return halo
    }

    // MARK: - Edge detection

    private static func sobelBinary(
        gray: [UInt8], w: Int, h: Int, thresh: Float
    ) -> [Bool] {
        var mag = [Float](repeating: 0, count: w * h)
        for y in 1..<(h - 1) {
            for x in 1..<(w - 1) {
                let gx =
                    -Int(gray[(y-1)*w+(x-1)])
                    - 2*Int(gray[y*w+(x-1)])
                    - Int(gray[(y+1)*w+(x-1)])
                    + Int(gray[(y-1)*w+(x+1)])
                    + 2*Int(gray[y*w+(x+1)])
                    + Int(gray[(y+1)*w+(x+1)])
                let gy =
                    -Int(gray[(y-1)*w+(x-1)])
                    - 2*Int(gray[(y-1)*w+x])
                    - Int(gray[(y-1)*w+(x+1)])
                    + Int(gray[(y+1)*w+(x-1)])
                    + 2*Int(gray[(y+1)*w+x])
                    + Int(gray[(y+1)*w+(x+1)])
                mag[y * w + x] = Float(gx * gx + gy * gy)
            }
        }
        let maxV = mag.max() ?? 1
        guard maxV > 0 else {
            return [Bool](repeating: false, count: w * h)
        }
        let t2 = thresh * thresh * maxV
        return mag.map { $0 > t2 }
    }

    // MARK: - TURBO colormap approximation

    private static func turbo(_ t: Float) -> (Float, Float, Float) {
        let c = max(0, min(1, t))
        let r: Float, g: Float, b: Float
        if c < 0.25 {
            let s = c / 0.25
            r = 48 + s * 21;  g = 18 + s * 199;  b = 59 + s * 179
        } else if c < 0.5 {
            let s = (c - 0.25) / 0.25
            r = 69 + s * 53;  g = 217 + s * 38;  b = 238 - s * 196
        } else if c < 0.75 {
            let s = (c - 0.5) / 0.25
            r = 122 + s * 132; g = 255 - s * 89; b = 42 - s * 26
        } else {
            let s = (c - 0.75) / 0.25
            r = 254 - s * 132; g = 166 - s * 162; b = 16 - s * 13
        }
        return (r, g, b)
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
        return stride(from: 0, to: buf.count, by: 4).map {
            RGBA(
                r: buf[$0], g: buf[$0+1],
                b: buf[$0+2], a: buf[$0+3]
            )
        }
    }

    private static func compositeHStack(
        left: [RGBA], right: [RGBA], w: Int, h: Int
    ) -> UIImage? {
        let divW = 4
        let totalW = w * 2 + divW
        var pixels = [UInt8](repeating: 0, count: totalW * h * 4)

        for y in 0..<h {
            for x in 0..<w {
                let src = left[y * w + x]
                let dst = (y * totalW + x) * 4
                pixels[dst] = src.r; pixels[dst+1] = src.g
                pixels[dst+2] = src.b; pixels[dst+3] = 255
            }
            for x in w..<(w + divW) {
                let dst = (y * totalW + x) * 4
                pixels[dst] = 0; pixels[dst+1] = 255
                pixels[dst+2] = 255; pixels[dst+3] = 255
            }
            for x in 0..<w {
                let src = right[y * w + x]
                let dst = (y * totalW + w + divW + x) * 4
                pixels[dst] = src.r; pixels[dst+1] = src.g
                pixels[dst+2] = src.b; pixels[dst+3] = 255
            }
        }

        guard let provider = CGDataProvider(
            data: Data(pixels) as CFData
        ) else { return nil }
        guard let cg = CGImage(
            width: totalW, height: h,
            bitsPerComponent: 8, bitsPerPixel: 32,
            bytesPerRow: totalW * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGBitmapInfo(
                rawValue: CGImageAlphaInfo.premultipliedLast.rawValue
            ),
            provider: provider,
            decode: nil, shouldInterpolate: false,
            intent: .defaultIntent
        ) else { return nil }
        // Rotate 90° CW for portrait display — depth buffer data
        // is always landscape but phone is held in portrait.
        return UIImage(cgImage: cg, scale: 1.0, orientation: .right)
    }
}
