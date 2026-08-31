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

        // Apply face rotation
        var tileW2C = simd_float4x4(1.0)
        let faceRot4 = simd_float4x4(simd_float3x3(
            faceRotation.columns.0,
            faceRotation.columns.1,
            faceRotation.columns.2
        ))
        tileW2C = faceRot4 * insta360W2C

        // Apply ARKit → COLMAP coordinate transform
        tileW2C = COLMAPExporter.arkitToCOLMAP * tileW2C

        let rotation = simd_float3x3(
            SIMD3(tileW2C.columns.0.x, tileW2C.columns.0.y, tileW2C.columns.0.z),
            SIMD3(tileW2C.columns.1.x, tileW2C.columns.1.y, tileW2C.columns.1.z),
            SIMD3(tileW2C.columns.2.x, tileW2C.columns.2.y, tileW2C.columns.2.z)
        )
        let quat = COLMAPExporter.rotationToQuaternion(rotation)
        let translation = SIMD3(tileW2C.columns.3.x, tileW2C.columns.3.y, tileW2C.columns.3.z)

        return (quat, translation)
    }
}
