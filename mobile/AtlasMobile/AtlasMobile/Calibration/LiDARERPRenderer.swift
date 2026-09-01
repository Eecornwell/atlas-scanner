import UIKit
import simd

/// Projects iPhone LiDAR depth into an ERP-sized grayscale UIImage.
/// Used by CalibrationManager (as input to FeatureMatcher) and
/// CalibrationOverlayRenderer (for the verification overlay).
///
/// Mirrors calibrate_extrinsic.py depth_to_erp_intensity().
enum LiDARERPRenderer {

    /// Returns a grayscale UIImage in ERP projection, sized to match the
    /// Insta360 ERP (taken from frame.erpImage), or nil on failure.
    static func render(frame: CalibrationFrame, extrinsic: RigidTransform) -> UIImage? {
        let erpW = Int(frame.erpImage.size.width)
        let erpH = Int(frame.erpImage.size.height)
        guard erpW > 0, erpH > 0 else { return nil }

        let pixels = project(frame: frame, extrinsic: extrinsic, erpW: erpW, erpH: erpH)

        // Render grayscale pixels into a UIImage
        var rgba = [UInt8](repeating: 0, count: erpW * erpH * 4)
        for i in 0..<(erpW * erpH) {
            let v = pixels[i]
            rgba[i*4]   = v
            rgba[i*4+1] = v
            rgba[i*4+2] = v
            rgba[i*4+3] = 255
        }
        let provider = CGDataProvider(data: Data(rgba) as CFData)!
        guard let cg = CGImage(
            width: erpW, height: erpH,
            bitsPerComponent: 8, bitsPerPixel: 32,
            bytesPerRow: erpW * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue),
            provider: provider,
            decode: nil, shouldInterpolate: false,
            intent: .defaultIntent
        ) else { return nil }
        return UIImage(cgImage: cg)
    }

    /// Returns raw uint8 pixel array (grayscale, depth-coloured) in ERP layout.
    static func project(
        frame: CalibrationFrame,
        extrinsic: RigidTransform,
        erpW: Int,
        erpH: Int
    ) -> [UInt8] {
        let T = extrinsic.toMatrix()
        let R = simd_float3x3(
            SIMD3(T.columns.0.x, T.columns.0.y, T.columns.0.z),
            SIMD3(T.columns.1.x, T.columns.1.y, T.columns.1.z),
            SIMD3(T.columns.2.x, T.columns.2.y, T.columns.2.z)
        )

        let scaleX = Float(frame.depthW) / Float(frame.imageW)
        let scaleY = Float(frame.depthH) / Float(frame.imageH)
        let fx = frame.intrinsics[0][0] * scaleX
        let fy = frame.intrinsics[1][1] * scaleY
        let cx = frame.intrinsics[2][0] * scaleX
        let cy = frame.intrinsics[2][1] * scaleY

        var img = [UInt8](repeating: 0, count: erpW * erpH)

        for row in 0..<frame.depthH {
            for col in 0..<frame.depthW {
                let z = frame.depth[row * frame.depthW + col]
                guard z > 0.15 && z < 5.0 else { continue }
                let x = (Float(col) - cx) / fx * z
                let y = (Float(row) - cy) / fy * z
                let pt = R * SIMD3(x, y, z)
                let norm = simd_length(pt)
                guard norm > 0.05 else { continue }
                let b = pt / norm

                // ERP projection: lat = -asin(Y), lon = atan2(X, Z)
                let lat = -asin(max(-1, min(1, b.y)))
                let lon = atan2(b.x, b.z)
                let u = Int(Float(erpW) * (0.5 + lon / (2 * .pi))) % erpW
                let v = max(0, min(erpH - 1, Int(Float(erpH) * (0.5 - lat / .pi))))
                guard u >= 0 else { continue }

                let intensity = UInt8(max(30, min(255, Int(255 * (1 - z / 5.0)))))
                for dy in -1...1 {
                    for dx in -1...1 {
                        let pu = (u + dx + erpW) % erpW
                        let pv = max(0, min(erpH - 1, v + dy))
                        img[pv * erpW + pu] = max(img[pv * erpW + pu], intensity)
                    }
                }
            }
        }
        return img
    }
}
