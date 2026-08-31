import CoreImage
import simd

/// Slices equirectangular panoramas into perspective face tiles.
/// Port of atlas-scanner's ERP → perspective tile extraction.
final class ERPTileSlicer {
    let faceCount: Int
    let tileFov: Double
    let tileSize: Int

    /// Pre-computed rotation matrices for each face direction.
    let faceRotations: [simd_float3x3]

    init(faceCount: Int = 8, tileFov: Double = 90, tileSize: Int = 1024) {
        self.faceCount = faceCount
        self.tileFov = tileFov
        self.tileSize = tileSize
        self.faceRotations = Self.computeFaceRotations(faceCount: faceCount)
    }

    /// Extracts perspective tiles from an equirectangular image.
    func sliceTiles(from erpImage: CIImage) -> [(faceIndex: Int, image: CIImage, rotation: simd_float3x3)] {
        // TODO: Implement ERP → perspective reprojection
        // For each face:
        //   1. Compute sampling grid in perspective image
        //   2. Map each pixel to spherical coordinates
        //   3. Apply face rotation
        //   4. Convert to ERP coordinates
        //   5. Sample from source image
        // Can use Metal/vImage for GPU acceleration on-device
        return []
    }

    /// Slices a mask in ERP projection into per-face tile masks.
    func sliceMask(from erpMask: CIImage) -> [(faceIndex: Int, mask: CIImage)] {
        // TODO: Same projection as sliceTiles but for single-channel mask
        return []
    }

    /// Computes the SIMPLE_PINHOLE intrinsics for a face tile.
    func tileIntrinsics() -> (f: Float, cx: Float, cy: Float) {
        let f = Float(tileSize) / (2.0 * tan(Float(tileFov / 2.0) * .pi / 180.0))
        let cx = Float(tileSize) / 2.0
        let cy = Float(tileSize) / 2.0
        return (f, cx, cy)
    }

    private static func computeFaceRotations(faceCount: Int) -> [simd_float3x3] {
        // TODO: Compute evenly-spaced rotations around vertical axis
        // plus ceiling face, matching atlas-scanner's face layout
        // face_00 = ceiling, face_01..07 = horizontal ring
        return (0..<faceCount).map { _ in matrix_identity_float3x3 }
    }
}
