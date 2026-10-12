import Foundation
import UIKit
import simd

/// LAB-space color profile matching the legacy color_normalize.py approach.
/// iPhone is the reference; Insta360 is the secondary camera to correct.
struct ColorProfile: Codable {
    /// Luminance gain: L' = L * lGain (clamped 0.7–1.5)
    let lGain: Float
    /// a* color-cast offset: a' = a + aOffset
    let aOffset: Float
    /// b* color-cast offset: b' = b + bOffset
    let bOffset: Float
}

enum ColorProfileCalibrator {

    /// Computes a color profile by comparing the iPhone image (reference)
    /// against the overlapping region of the Insta360 ERP (secondary).
    ///
    /// Projects iPhone pixels into ERP space using the calibrated extrinsic,
    /// converts corresponding pixel pairs to CIELAB, then computes gain/offset
    /// statistics. Near-black (<20) and overexposed (>245) pixels are excluded.
    static func compute(
        iphoneImage: UIImage,
        erpImage: UIImage,
        extrinsic: RigidTransform,
        frame: CalibrationFrame,
        mask: UIImage? = nil
    ) -> ColorProfile? {
        guard let iphoneCG = iphoneImage.cgImage,
              let erpCG = erpImage.cgImage else { return nil }

        let iW = iphoneCG.width, iH = iphoneCG.height
        let eW = erpCG.width, eH = erpCG.height

        guard let iPixels = extractRGBA(from: iphoneCG),
              let ePixels = extractRGBA(from: erpCG) else { return nil }

        let maskBool: [Bool]?
        if let maskCG = mask?.cgImage {
            maskBool = extractGrayMask(from: maskCG, width: eW, height: eH)
        } else {
            maskBool = nil
        }

        let T = ExtrinsicOptimizer.seedToCameraMatrix(extrinsic)
        let R = simd_float3x3(
            SIMD3(T.columns.0.x, T.columns.0.y, T.columns.0.z),
            SIMD3(T.columns.1.x, T.columns.1.y, T.columns.1.z),
            SIMD3(T.columns.2.x, T.columns.2.y, T.columns.2.z)
        )

        let di = frame.depthIntrinsics()
        let dW = frame.depthW, dH = frame.depthH
        let scaleToImgX = Float(iW) / Float(dW)
        let scaleToImgY = Float(iH) / Float(dH)

        var iLSum: Float = 0, iaSum: Float = 0, ibSum: Float = 0
        var eLSum: Float = 0, eaSum: Float = 0, ebSum: Float = 0
        var count = 0

        let step = 2
        for row in stride(from: 1, to: dH - 1, by: step) {
            for col in stride(from: 1, to: dW - 1, by: step) {
                let iCol = min(iW - 1, Int(Float(col) * scaleToImgX))
                let iRow = min(iH - 1, Int(Float(row) * scaleToImgY))

                let xCam = (Float(col) - di.cx) / di.fx
                let yCam = (Float(row) - di.cy) / di.fy
                let ray = SIMD3<Float>(-xCam, -yCam, 1.0)

                let rotated = ExtrinsicOptimizer.erpCorrection
                    * simd_normalize(R * ray)
                let lat = -asin(max(-1, min(1, rotated.y)))
                let lon = atan2(rotated.x, rotated.z)
                var eCol = Int(Float(eW) * (0.5 + lon / (2 * .pi)))
                let eRow = max(
                    0, min(eH - 1,
                           Int(Float(eH) * (0.5 - lat / .pi)))
                )
                eCol = ((eCol % eW) + eW) % eW

                if let m = maskBool, m[eRow * eW + eCol] { continue }

                let iIdx = (iRow * iW + iCol) * 4
                let iR = Float(iPixels[iIdx])
                let iG = Float(iPixels[iIdx + 1])
                let iB = Float(iPixels[iIdx + 2])

                let eIdx = (eRow * eW + eCol) * 4
                let eR = Float(ePixels[eIdx])
                let eG = Float(ePixels[eIdx + 1])
                let eB = Float(ePixels[eIdx + 2])

                let iMax = max(iR, max(iG, iB))
                let iMin = min(iR, min(iG, iB))
                let eMax = max(eR, max(eG, eB))
                let eMin = min(eR, min(eG, eB))
                if iMin < 20 || iMax > 245 { continue }
                if eMin < 20 || eMax > 245 { continue }

                let (iL, ia, ib) = rgbToLAB(iR, iG, iB)
                let (eL, ea, eb) = rgbToLAB(eR, eG, eB)

                iLSum += iL; iaSum += ia; ibSum += ib
                eLSum += eL; eaSum += ea; ebSum += eb
                count += 1
            }
        }

        guard count > 100 else {
            print("[Color] Too few valid pixel pairs: \(count)")
            return nil
        }

        let n = Float(count)
        let lGain = max(0.7, min(1.5,
            (iLSum / n) / max(eLSum / n, 0.01)))
        let aOff = iaSum / n - eaSum / n
        let bOff = ibSum / n - ebSum / n

        print("[Color] \(count) pairs:"
            + " L_gain=\(String(format: "%.3f", lGain))"
            + " a_off=\(String(format: "%.2f", aOff))"
            + " b_off=\(String(format: "%.2f", bOff))")

        return ColorProfile(
            lGain: lGain, aOffset: aOff, bOffset: bOff
        )
    }

    /// Applies the color profile to an image (corrects Insta360 toward iPhone colors).
    static func apply(
        _ profile: ColorProfile, to image: UIImage
    ) -> UIImage {
        guard let cg = image.cgImage else { return image }
        let w = cg.width, h = cg.height
        var pixels = [UInt8](repeating: 0, count: w * h * 4)
        guard let ctx = CGContext(
            data: &pixels, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return image }
        ctx.draw(cg, in: CGRect(x: 0, y: 0, width: w, height: h))

        for i in stride(from: 0, to: pixels.count, by: 4) {
            let r = Float(pixels[i])
            let g = Float(pixels[i + 1])
            let b = Float(pixels[i + 2])

            var (l, a, bVal) = rgbToLAB(r, g, b)
            l *= profile.lGain
            a += profile.aOffset
            bVal += profile.bOffset
            let (rOut, gOut, bOut) = labToRGB(l, a, bVal)

            pixels[i]     = UInt8(max(0, min(255, rOut)))
            pixels[i + 1] = UInt8(max(0, min(255, gOut)))
            pixels[i + 2] = UInt8(max(0, min(255, bOut)))
        }

        guard let outCtx = CGContext(
            data: &pixels, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ), let result = outCtx.makeImage() else { return image }
        return UIImage(cgImage: result)
    }

    // MARK: - Persistence

    private static let profileURL: URL? = {
        FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first?.appendingPathComponent(
            "atlas_sessions/color_profile.json"
        )
    }()

    static func save(_ profile: ColorProfile) {
        guard let url = profileURL else { return }
        try? FileManager.default.createDirectory(
            at: url.deletingLastPathComponent(),
            withIntermediateDirectories: true
        )
        let encoder = JSONEncoder()
        encoder.outputFormatting = .prettyPrinted
        if let data = try? encoder.encode(profile) {
            try? data.write(to: url)
            print("[Color] saved to \(url.path)")
        }
    }

    static func load() -> ColorProfile? {
        guard let url = profileURL,
              let data = try? Data(contentsOf: url)
        else { return nil }
        return try? JSONDecoder().decode(
            ColorProfile.self, from: data
        )
    }

    // MARK: - sRGB ↔ CIELAB (D65)

    static func rgbToLAB(
        _ r: Float, _ g: Float, _ b: Float
    ) -> (Float, Float, Float) {
        func linearize(_ c: Float) -> Float {
            let v = c / 255.0
            return v <= 0.04045
                ? v / 12.92
                : pow((v + 0.055) / 1.055, 2.4)
        }
        let rl = linearize(r)
        let gl = linearize(g)
        let bl = linearize(b)

        let x = 0.4124564 * rl + 0.3575761 * gl + 0.1804375 * bl
        let y = 0.2126729 * rl + 0.7151522 * gl + 0.0721750 * bl
        let z = 0.0193339 * rl + 0.1191920 * gl + 0.9503041 * bl

        let xn: Float = 0.95047
        let yn: Float = 1.0
        let zn: Float = 1.08883

        func f(_ t: Float) -> Float {
            t > 0.008856
                ? pow(t, 1.0 / 3.0)
                : 7.787 * t + 16.0 / 116.0
        }

        let fx = f(x / xn), fy = f(y / yn), fz = f(z / zn)
        return (116.0 * fy - 16.0,
                500.0 * (fx - fy),
                200.0 * (fy - fz))
    }

    static func labToRGB(
        _ L: Float, _ a: Float, _ b: Float
    ) -> (Float, Float, Float) {
        let xn: Float = 0.95047
        let yn: Float = 1.0
        let zn: Float = 1.08883

        let fy = (L + 16.0) / 116.0
        let fx = a / 500.0 + fy
        let fz = fy - b / 200.0

        func finv(_ t: Float) -> Float {
            let t3 = t * t * t
            return t3 > 0.008856
                ? t3
                : (t - 16.0 / 116.0) / 7.787
        }

        let x = xn * finv(fx)
        let y = yn * finv(fy)
        let z = zn * finv(fz)

        let rl =  3.2404542 * x - 1.5371385 * y - 0.4985314 * z
        let gl = -0.9692660 * x + 1.8760108 * y + 0.0415560 * z
        let bl =  0.0556434 * x - 0.2040259 * y + 1.0572252 * z

        func gammaEncode(_ c: Float) -> Float {
            let v = max(0, c)
            let g = v <= 0.0031308
                ? 12.92 * v
                : 1.055 * pow(v, 1.0 / 2.4) - 0.055
            return g * 255.0
        }

        return (gammaEncode(rl), gammaEncode(gl), gammaEncode(bl))
    }

    // MARK: - Pixel extraction

    private static func extractRGBA(
        from cgImage: CGImage
    ) -> [UInt8]? {
        let w = cgImage.width, h = cgImage.height
        var pixels = [UInt8](repeating: 0, count: w * h * 4)
        guard let ctx = CGContext(
            data: &pixels, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return nil }
        ctx.draw(
            cgImage,
            in: CGRect(x: 0, y: 0, width: w, height: h)
        )
        return pixels
    }

    private static func extractGrayMask(
        from cgImage: CGImage, width: Int, height: Int
    ) -> [Bool] {
        var gray = [UInt8](repeating: 0, count: width * height)
        guard let ctx = CGContext(
            data: &gray, width: width, height: height,
            bitsPerComponent: 8, bytesPerRow: width,
            space: CGColorSpaceCreateDeviceGray(),
            bitmapInfo: CGImageAlphaInfo.none.rawValue
        ) else { return [Bool](repeating: false, count: width * height) }
        ctx.draw(
            cgImage,
            in: CGRect(x: 0, y: 0, width: width, height: height)
        )
        return gray.map { $0 > 128 }
    }
}
