import Foundation
import simd

struct RigidTransform: Codable {
    let roll: Double
    let pitch: Double
    let yaw: Double
    let x: Double
    let y: Double
    let z: Double

    /// Converts RPY + XYZ to a 4x4 homogeneous transform matrix.
    func toMatrix() -> simd_float4x4 {
        let r = Float(roll * .pi / 180)
        let p = Float(pitch * .pi / 180)
        let y = Float(yaw * .pi / 180)

        let cr = cos(r); let sr = sin(r)
        let cp = cos(p); let sp = sin(p)
        let cy = cos(y); let sy = sin(y)

        let rotation = simd_float3x3(rows: [
            SIMD3(cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            SIMD3(sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            SIMD3(-sp, cp * sr, cp * cr)
        ])

        var matrix = simd_float4x4(1.0)
        matrix.columns.0 = SIMD4(rotation.columns.0, 0)
        matrix.columns.1 = SIMD4(rotation.columns.1, 0)
        matrix.columns.2 = SIMD4(rotation.columns.2, 0)
        matrix.columns.3 = SIMD4(Float(self.x), Float(self.y), Float(self.z), 1)

        return matrix
    }
}

struct CameraConfig: Codable {
    let id: String
    let model: String
    let serial: String
    let extrinsic: RigidTransform
    let mask: String?
    let faceCount: Int
    let tileFov: Double

    enum CodingKeys: String, CodingKey {
        case id, model, serial, extrinsic, mask
        case faceCount = "face_count"
        case tileFov = "tile_fov"
    }
}

struct IPhoneConfig: Codable {
    let mask: String?
}

struct MultiCameraConfig: Codable {
    let cameras: [CameraConfig]
    let iphone: IPhoneConfig

    static let `default` = MultiCameraConfig(
        cameras: [],
        iphone: IPhoneConfig(mask: nil)
    )

    static func load(from url: URL) throws -> MultiCameraConfig {
        let data = try Data(contentsOf: url)
        // TODO: Use a YAML parser (e.g., Yams) for full compatibility with atlas-scanner format
        return try JSONDecoder().decode(MultiCameraConfig.self, from: data)
    }
}
