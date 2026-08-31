import Foundation
import simd

/// Exports session data to COLMAP binary format.
/// Mirrors atlas-scanner's panorama_sfm_colmap.py output structure.
final class COLMAPExporter {

    /// COLMAP camera model IDs
    enum CameraModel: UInt32 {
        case simplePinhole = 0  // f, cx, cy — for Insta360 ERP tiles
        case pinhole = 1        // fx, fy, cx, cy — for iPhone
    }

    private let sessionDirectory: URL
    private let colmapDir: URL

    init(sessionDirectory: URL) {
        self.sessionDirectory = sessionDirectory
        self.colmapDir = sessionDirectory.appendingPathComponent("colmap")
    }

    func export(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig
    ) throws {
        try writeCamerasBin(cameraConfig: cameraConfig)
        try writeImagesBin(scans: scans, cameraConfig: cameraConfig)
        try writePoints3DBin()
        try writeRigsBin(cameraConfig: cameraConfig)
    }

    // MARK: - cameras.bin

    private func writeCamerasBin(cameraConfig: MultiCameraConfig) throws {
        let url = colmapDir.appendingPathComponent("sparse/0/cameras.bin")
        var data = Data()

        // Camera count: iPhone + (N cameras * face_count each)
        let totalCameras = 1 + cameraConfig.cameras.reduce(0) { $0 + $1.faceCount }
        data.appendLittleEndian(UInt64(totalCameras))

        // iPhone camera (PINHOLE model)
        // TODO: Read actual intrinsics from captured scan data
        data.appendLittleEndian(UInt32(1))  // camera_id
        data.appendLittleEndian(CameraModel.pinhole.rawValue)
        data.appendLittleEndian(UInt64(4032))  // width
        data.appendLittleEndian(UInt64(3024))  // height
        // TODO: Write fx, fy, cx, cy as Float64

        // Per Insta360 camera face tiles (SIMPLE_PINHOLE)
        // TODO: Generate camera entries for each face tile

        try data.write(to: url)
    }

    // MARK: - images.bin

    private func writeImagesBin(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig
    ) throws {
        let url = colmapDir.appendingPathComponent("sparse/0/images.bin")
        var data = Data()

        // TODO: Write image entries
        // For each scan:
        //   1. iPhone image with ARKit pose (converted to COLMAP frame)
        //   2. Per Insta360 camera, per face tile:
        //      pose = tile_rotation @ T_insta360_iphone @ T_iphone_world
        //      Convert to quaternion (wxyz) + translation

        try data.write(to: url)
    }

    // MARK: - points3D.bin

    private func writePoints3DBin() throws {
        let url = colmapDir.appendingPathComponent("sparse/0/points3D.bin")
        var data = Data()
        data.appendLittleEndian(UInt64(0))  // empty initially
        try data.write(to: url)
    }

    // MARK: - rigs.bin

    private func writeRigsBin(cameraConfig: MultiCameraConfig) throws {
        guard !cameraConfig.cameras.isEmpty else { return }
        let url = colmapDir.appendingPathComponent("sparse/0/rigs.bin")
        // TODO: Write rig definition (iPhone body + N camera extrinsics)
        try Data().write(to: url)
    }

    // MARK: - Coordinate transforms

    /// Converts ARKit Y-up right-handed to COLMAP convention.
    static let arkitToCOLMAP = simd_float4x4(rows: [
        SIMD4(1, 0, 0, 0),
        SIMD4(0, -1, 0, 0),
        SIMD4(0, 0, -1, 0),
        SIMD4(0, 0, 0, 1)
    ])

    /// Converts a 3x3 rotation matrix to COLMAP quaternion (wxyz).
    static func rotationToQuaternion(_ r: simd_float3x3) -> SIMD4<Float> {
        let quat = simd_quatf(r)
        return SIMD4(quat.real, quat.imag.x, quat.imag.y, quat.imag.z)
    }
}

struct ScanExportData {
    let scanIndex: Int
    let arkitPose: simd_float4x4
    let intrinsics: simd_float3x3
    let iphoneImagePath: String
    let insta360ImagePaths: [String: String]  // cameraId → ERP path
}

extension Data {
    mutating func appendLittleEndian<T: FixedWidthInteger>(_ value: T) {
        var v = value.littleEndian
        append(Data(bytes: &v, count: MemoryLayout<T>.size))
    }

    mutating func appendLittleEndian(_ value: Float64) {
        var v = value
        append(Data(bytes: &v, count: MemoryLayout<Float64>.size))
    }
}
