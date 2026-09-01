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

    /// Pre-computed (u, v) sampling maps per face, lazily built on first use.
    private var samplingMaps: [(u: [Float], v: [Float])]?

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

    /// 8 equatorial face rotations matching atlas panorama_sfm_colmap.py FACES_CAM_FROM_PANO.
    /// Insta360 SDK ERP has forward at top (v=0); pre-compose R_y(+90°) to land
    /// equatorial faces on the actual horizon band — same correction as atlas.
    static func computeFaceRotations() -> [simd_float3x3] {
        let instaCorrection = simd_quatf(angle: .pi / 2, axis: SIMD3(0, 1, 0))
        return (0..<8).map { i in
            let yawRad = Float(i) * .pi / 4.0   // 45° steps
            let faceQuat = simd_quatf(angle: -yawRad, axis: SIMD3(0, 1, 0))
            return simd_float3x3(faceQuat * instaCorrection)
        }
    }

    // MARK: - Sampling map construction

    private func buildSamplingMaps(erpW: Int, erpH: Int) -> [(u: [Float], v: [Float])] {
        // Cache maps keyed to ERP dimensions — they only depend on tileSize/fov/erpSize
        if let cached = samplingMaps { return cached }

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
            // Rotate ray directions: r_pano = R * ray_cam
            // R is column-major simd_float3x3
            var rx = [Float](repeating: 0, count: n)
            var ry = [Float](repeating: 0, count: n)
            var rz = [Float](repeating: 0, count: n)

            for i in 0..<n {
                let ray = SIMD3<Float>(px[i], py[i], pz[i])
                let rot = R * ray
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
        return maps
    }

    // MARK: - vImage remap

    /// Bilinear remap of a CGImage using pre-computed float (u, v) maps.
    private func remap(source: CGImage, uMap: [Float], vMap: [Float]) -> CGImage? {
        let srcW = source.width, srcH = source.height
        let bytesPerPixel = 4
        let srcStride = srcW * bytesPerPixel

        // Render source into RGBA8 buffer
        guard let srcCtx = CGContext(
            data: nil,
            width: srcW, height: srcH,
            bitsPerComponent: 8, bytesPerRow: srcStride,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return nil }
        srcCtx.draw(source, in: CGRect(x: 0, y: 0, width: srcW, height: srcH))
        guard let srcBytes = srcCtx.data else { return nil }
        let src = srcBytes.bindMemory(to: UInt8.self, capacity: srcH * srcStride)

        let dstStride = tileSize * bytesPerPixel
        let dstBuf = UnsafeMutablePointer<UInt8>.allocate(capacity: tileSize * dstStride)
        defer { dstBuf.deallocate() }

        let n = tileSize * tileSize
        for i in 0..<n {
            let u = uMap[i], v = vMap[i]
            // Bilinear interpolation
            let x0 = Int(u), y0 = Int(v)
            let x1 = min(x0 + 1, srcW - 1)
            let y1 = min(y0 + 1, srcH - 1)
            let fx = u - Float(x0), fy = v - Float(y0)
            let x0c = max(0, min(x0, srcW - 1))
            let y0c = max(0, min(y0, srcH - 1))

            let p00 = src + y0c * srcStride + x0c * bytesPerPixel
            let p10 = src + y0c * srcStride + x1  * bytesPerPixel
            let p01 = src + y1  * srcStride + x0c * bytesPerPixel
            let p11 = src + y1  * srcStride + x1  * bytesPerPixel

            let dst = dstBuf + i * bytesPerPixel
            for c in 0..<4 {
                let v00 = Float(p00[c]), v10 = Float(p10[c])
                let v01 = Float(p01[c]), v11 = Float(p11[c])
                let top = v00 + (v10 - v00) * fx
                let bot = v01 + (v11 - v01) * fx
                dst[c] = UInt8(clamping: Int(top + (bot - top) * fy))
            }
        }

        guard let dstCtx = CGContext(
            data: dstBuf,
            width: tileSize, height: tileSize,
            bitsPerComponent: 8, bytesPerRow: dstStride,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return nil }

        return dstCtx.makeImage()
    }
}
