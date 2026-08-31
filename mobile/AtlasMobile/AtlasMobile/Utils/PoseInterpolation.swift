import simd

/// Pose interpolation utilities matching atlas-scanner's Slerp + lerp approach.
enum PoseInterpolation {
    /// Interpolates between two 4x4 pose matrices using slerp (rotation) and lerp (translation).
    static func interpolate(from a: simd_float4x4, to b: simd_float4x4, t: Float) -> simd_float4x4 {
        let rotA = simd_quatf(simd_float3x3(
            SIMD3(a.columns.0.x, a.columns.0.y, a.columns.0.z),
            SIMD3(a.columns.1.x, a.columns.1.y, a.columns.1.z),
            SIMD3(a.columns.2.x, a.columns.2.y, a.columns.2.z)
        ))
        let rotB = simd_quatf(simd_float3x3(
            SIMD3(b.columns.0.x, b.columns.0.y, b.columns.0.z),
            SIMD3(b.columns.1.x, b.columns.1.y, b.columns.1.z),
            SIMD3(b.columns.2.x, b.columns.2.y, b.columns.2.z)
        ))

        let interpRot = simd_slerp(rotA, rotB, t)

        let transA = SIMD3(a.columns.3.x, a.columns.3.y, a.columns.3.z)
        let transB = SIMD3(b.columns.3.x, b.columns.3.y, b.columns.3.z)
        let interpTrans = mix(transA, transB, t: t)

        let rotMatrix = simd_float3x3(interpRot)
        var result = simd_float4x4(1.0)
        result.columns.0 = SIMD4(rotMatrix.columns.0, 0)
        result.columns.1 = SIMD4(rotMatrix.columns.1, 0)
        result.columns.2 = SIMD4(rotMatrix.columns.2, 0)
        result.columns.3 = SIMD4(interpTrans, 1)

        return result
    }
}
