import Accelerate
import CoreGraphics
import UIKit
import simd

/// Slices equirectangular panoramas into perspective face tiles on-device.
/// Face layout and rotation math match atlas-scanner's panorama_sfm_colmap.py
/// and COLMAPExporter.computeFaceRotations exactly.
final class ERPTileSlicer {
    let tileSize: Int
    let tileFovDeg: Float

    /// Pre-computed cam-from-pano rotation matrices for each face.
    let faceRotations: [simd_float3x3]

    /// Pre-computed (u, v) sampling maps per face, keyed by ERP dimensions.
    private var samplingMaps: [(u: [Float], v: [Float])]?
    private var cachedW: Int = 0
    private var cachedH: Int = 0

    init(tileSize: Int = 1024, tileFovDeg: Float = 65.0) {
        self.tileSize = tileSize
        self.tileFovDeg = tileFovDeg
        self.faceRotations = Self.computeFaceRotations()
    }

    // MARK: - Public

    /// Slices an ERP UIImage into perspective face tiles.
    /// Returns (faceIndex, tile) only for faces whose sampling map is valid.
    func sliceTiles(from erpImage: UIImage) -> [(faceIndex: Int, image: UIImage)] {
        guard let cgErp = erpImage.cgImage else { return [] }
        let maps = buildSamplingMaps(erpW: cgErp.width, erpH: cgErp.height)
        var results: [(Int, UIImage)] = []
        for (i, map) in maps.enumerated() {
            if let tile = remap(source: cgErp, uMap: map.u, vMap: map.v) {
                results.append((i, UIImage(cgImage: tile)))
            }
        }
        return results
    }

    /// Slices a single face tile from an ERP image. Lower peak memory
    /// than `sliceTiles` when tiles are written to disk one at a time.
    func sliceTile(at faceIndex: Int, from erpImage: UIImage) -> UIImage? {
        guard let cgErp = erpImage.cgImage else { return nil }
        let maps = buildSamplingMaps(erpW: cgErp.width, erpH: cgErp.height)
        guard faceIndex < maps.count else { return nil }
        let map = maps[faceIndex]
        guard let tile = remap(source: cgErp, uMap: map.u, vMap: map.v)
        else { return nil }
        return UIImage(cgImage: tile)
    }

    /// Slices a single-channel ERP mask into per-face tile masks.
    func sliceMask(from erpMask: UIImage) -> [(faceIndex: Int, mask: UIImage)] {
        guard let cgMask = erpMask.cgImage else { return [] }
        let maps = buildSamplingMaps(erpW: cgMask.width, erpH: cgMask.height)
        var results: [(Int, UIImage)] = []
        for (i, map) in maps.enumerated() {
            if let tile = remap(source: cgMask, uMap: map.u, vMap: map.v) {
                results.append((i, UIImage(cgImage: tile)))
            }
        }
        return results
    }

    /// SIMPLE_PINHOLE intrinsics for a face tile.
    func tileIntrinsics() -> (f: Float, cx: Float, cy: Float) {
        let f = Float(tileSize) / (2.0 * tan(tileFovDeg * .pi / 360.0))
        let c = Float(tileSize) / 2.0
        return (f, c, c)
    }

    // MARK: - Face rotations

    /// 8 equatorial face rotations (cam_from_pano) at 45° yaw steps.
    /// DualFisheyeStitcher produces standard ERP: lon=0 at center,
    /// v=0 at zenith, equator at v=H/2.
    static func computeFaceRotations() -> [simd_float3x3] {
        (0..<8).map { i in
            let yawRad = Float(i) * .pi / 4.0
            let faceQuat = simd_quatf(angle: -yawRad, axis: SIMD3(0, 1, 0))
            return simd_float3x3(faceQuat)
        }
    }

    // MARK: - Sampling map construction

    private func buildSamplingMaps(erpW: Int, erpH: Int) -> [(u: [Float], v: [Float])] {
        if let cached = samplingMaps, cachedW == erpW, cachedH == erpH {
            return cached
        }

        let n = tileSize * tileSize
        let (f, cx, cy) = tileIntrinsics()
        let W = Float(erpW), H = Float(erpH)

        // Pixel centre grid for the output tile
        var px = [Float](repeating: 0, count: n)
        var py = [Float](repeating: 0, count: n)
        for row in 0..<tileSize {
            for col in 0..<tileSize {
                px[row * tileSize + col] = (Float(col) + 0.5 - cx) / f
                py[row * tileSize + col] = (Float(row) + 0.5 - cy) / f
            }
        }
        // z = 1 for all rays
        var pz = [Float](repeating: 1.0, count: n)

        // Normalise to unit sphere
        var norms = [Float](repeating: 0, count: n)
        for i in 0..<n {
            norms[i] = sqrt(px[i]*px[i] + py[i]*py[i] + pz[i]*pz[i])
        }
        vDSP_vdiv(norms, 1, px, 1, &px, 1, vDSP_Length(n))
        vDSP_vdiv(norms, 1, py, 1, &py, 1, vDSP_Length(n))
        vDSP_vdiv(norms, 1, pz, 1, &pz, 1, vDSP_Length(n))

        var maps: [(u: [Float], v: [Float])] = []

        for R in faceRotations {
            // R is cam_from_pano; transpose to get pano_from_cam.
            // Matches Python: rays_pano = rays @ cam_from_pano (= cam_from_pano^T * ray)
            let Rt = R.transpose
            var rx = [Float](repeating: 0, count: n)
            var ry = [Float](repeating: 0, count: n)
            var rz = [Float](repeating: 0, count: n)

            for i in 0..<n {
                let ray = SIMD3<Float>(px[i], py[i], pz[i])
                let rot = Rt * ray
                rx[i] = rot.x
                ry[i] = rot.y
                rz[i] = rot.z
            }

            // Spherical → ERP pixel coordinates
            // yaw   = atan2(rx, rz)
            // pitch = -atan2(ry, sqrt(rx²+rz²))
            // u_erp = W * (0.5 + yaw / 2π)
            // v_erp = H * (0.5 - pitch / π)
            var uMap = [Float](repeating: 0, count: n)
            var vMap = [Float](repeating: 0, count: n)

            for i in 0..<n {
                let yaw   = atan2f(rx[i], rz[i])
                let horiz = sqrt(rx[i]*rx[i] + rz[i]*rz[i])
                let pitch = -atan2f(ry[i], horiz)
                uMap[i] = W * (0.5 + yaw   / (2 * .pi)) - 0.5
                vMap[i] = H * (0.5 - pitch / .pi)       - 0.5
                // Wrap u horizontally
                if uMap[i] < 0        { uMap[i] += W }
                if uMap[i] >= W       { uMap[i] -= W }
            }

            maps.append((u: uMap, v: vMap))
        }

        samplingMaps = maps
        cachedW = erpW
        cachedH = erpH
        return maps
    }

    // MARK: - Pre-extracted pixel path

    /// Extract ERP pixels in top-down layout matching the
    /// sampling maps. Call once, then pass the buffer to
    /// `sliceTile(at:srcPx:srcW:srcH:)` for each face.
    static func extractPixelsTopDown(
        _ image: UIImage
    ) -> (px: [UInt8], w: Int, h: Int)? {
        guard let cg = image.cgImage else { return nil }
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
        ctx.translateBy(x: 0, y: CGFloat(h))
        ctx.scaleBy(x: 1, y: -1)
        ctx.draw(
            cg,
            in: CGRect(x: 0, y: 0, width: w, height: h)
        )
        return (px, w, h)
    }

    /// Slice a single face from pre-extracted pixels.
    func sliceTile(
        at faceIndex: Int,
        srcPx: [UInt8], srcW: Int, srcH: Int
    ) -> UIImage? {
        let maps = buildSamplingMaps(
            erpW: srcW, erpH: srcH
        )
        guard faceIndex < maps.count else { return nil }
        let map = maps[faceIndex]
        guard let tile = remapPixels(
            srcPx: srcPx, srcW: srcW, srcH: srcH,
            uMap: map.u, vMap: map.v
        ) else { return nil }
        return UIImage(cgImage: tile)
    }

    // MARK: - vImage remap

    /// Bilinear remap of a CGImage using pre-computed float (u, v) maps.
    ///
    /// Draws the source into a buffer with a flipped CTM so row 0 = image
    /// top (standard top-down layout), matching the UV map convention.
    private func remap(source: CGImage, uMap: [Float], vMap: [Float]) -> CGImage? {
        let srcW = source.width, srcH = source.height
        let srcBPP = 4
        let srcStride = srcW * srcBPP

        var srcPx = [UInt8](repeating: 0, count: srcH * srcStride)
        guard let readCtx = CGContext(
            data: &srcPx, width: srcW, height: srcH,
            bitsPerComponent: 8, bytesPerRow: srcStride,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue
        ) else { return nil }
        readCtx.translateBy(x: 0, y: CGFloat(srcH))
        readCtx.scaleBy(x: 1, y: -1)
        readCtx.draw(source, in: CGRect(x: 0, y: 0, width: srcW, height: srcH))

        return remapPixels(
            srcPx: srcPx, srcW: srcW, srcH: srcH,
            uMap: uMap, vMap: vMap
        )
    }

    /// Bilinear remap from a pre-extracted top-down pixel buffer.
    private func remapPixels(
        srcPx: [UInt8], srcW: Int, srcH: Int,
        uMap: [Float], vMap: [Float]
    ) -> CGImage? {
        let srcBPP = 4
        let srcStride = srcW * srcBPP

        let dstBPP = 4
        let dstStride = tileSize * dstBPP
        var dstPx = [UInt8](repeating: 0, count: tileSize * dstStride)

        let n = tileSize * tileSize
        for i in 0..<n {
            let u = uMap[i], v = vMap[i]
            let x0 = Int(u), y0 = Int(v)
            let x1 = min(x0 + 1, srcW - 1)
            let y1 = min(y0 + 1, srcH - 1)
            let fx = u - Float(x0), fy = v - Float(y0)
            let x0c = max(0, min(x0, srcW - 1))
            let y0c = max(0, min(y0, srcH - 1))

            let off00 = y0c * srcStride + x0c * srcBPP
            let off10 = y0c * srcStride + x1  * srcBPP
            let off01 = y1  * srcStride + x0c * srcBPP
            let off11 = y1  * srcStride + x1  * srcBPP

            let dstOff = i * dstBPP
            for c in 0..<3 {
                let v00 = Float(srcPx[off00 + c])
                let v10 = Float(srcPx[off10 + c])
                let v01 = Float(srcPx[off01 + c])
                let v11 = Float(srcPx[off11 + c])
                let top = v00 + (v10 - v00) * fx
                let bot = v01 + (v11 - v01) * fx
                dstPx[dstOff + c] = UInt8(clamping: Int(top + (bot - top) * fy))
            }
            dstPx[dstOff + 3] = 255
        }

        guard let dstCtx = CGContext(
            data: &dstPx,
            width: tileSize, height: tileSize,
            bitsPerComponent: 8, bytesPerRow: dstStride,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue
        ) else { return nil }

        return dstCtx.makeImage()
    }
}
