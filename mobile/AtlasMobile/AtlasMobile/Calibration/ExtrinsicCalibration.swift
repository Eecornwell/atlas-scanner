import Foundation
import simd

/// Loads and applies extrinsic calibration for Insta360 ↔ iPhone rigid mount.
struct ExtrinsicCalibration {
    let cameraId: String
    let transform: RigidTransform
    private let matrix: simd_float4x4

    init(cameraId: String, transform: RigidTransform) {
        self.cameraId = cameraId
        self.transform = transform
        self.matrix = transform.toMatrix()
    }

    /// Computes the Insta360 camera world pose from the iPhone ARKit pose.
    /// T_insta360_world = T_insta360_iphone @ T_iphone_world
    func insta360Pose(from iphonePose: simd_float4x4) -> simd_float4x4 {
        matrix * iphonePose
    }

    /// Computes a face tile's world-to-camera transform for COLMAP export.
    /// T_tile_w2c = R_tile @ T_insta360_iphone @ inv(T_iphone_world)
    func tilePoseW2C(
        iphonePose: simd_float4x4,
        faceRotation: simd_float3x3
    ) -> (quaternionWXYZ: SIMD4<Float>, translation: SIMD3<Float>) {
        let insta360World = insta360Pose(from: iphonePose)
        let insta360W2C = insta360World.inverse

        let faceRot4 = simd_float4x4(
            SIMD4(faceRotation.columns.0, 0),
            SIMD4(faceRotation.columns.1, 0),
            SIMD4(faceRotation.columns.2, 0),
            SIMD4(0, 0, 0, 1)
        )
        var tileW2C = faceRot4 * insta360W2C

        let coordTransform = simd_float4x4(
            SIMD4(COLMAPExporter.arkitToColmap.columns.0, 0),
            SIMD4(COLMAPExporter.arkitToColmap.columns.1, 0),
            SIMD4(COLMAPExporter.arkitToColmap.columns.2, 0),
            SIMD4(0, 0, 0, 1)
        )
        tileW2C = coordTransform * tileW2C

        let rotation = simd_float3x3(
            SIMD3(tileW2C.columns.0.x, tileW2C.columns.0.y, tileW2C.columns.0.z),
            SIMD3(tileW2C.columns.1.x, tileW2C.columns.1.y, tileW2C.columns.1.z),
            SIMD3(tileW2C.columns.2.x, tileW2C.columns.2.y, tileW2C.columns.2.z)
        )
        let quatArray = COLMAPExporter.rotationToQuatWXYZ(rotation)
        let quat = SIMD4<Float>(quatArray[0], quatArray[1], quatArray[2], quatArray[3])
        let translation = SIMD3(tileW2C.columns.3.x, tileW2C.columns.3.y, tileW2C.columns.3.z)

        return (quat, translation)
    }
}
