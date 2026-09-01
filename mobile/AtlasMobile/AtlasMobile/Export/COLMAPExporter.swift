import Foundation
import simd
import ARKit

/// Exports a captured session to COLMAP binary format on-device.
/// Mirrors assemble_colmap.py — produces cameras.bin, images.bin, points3D.bin.
/// ERP tile slicing and depth image generation are deferred to the offline pipeline.
final class COLMAPExporter {

    // COLMAP camera model IDs
    private enum CameraModel: Int32 {
        case simplePinhole = 0  // f, cx, cy — Insta360 face tiles
        case pinhole = 1        // fx, fy, cx, cy — iPhone
    }

    // ARKit Y-up → COLMAP Y-down: flip Y and Z
    // Matches R_ARKIT2COLMAP = diag(1,-1,-1) in assemble_colmap.py
    static let arkitToColmap = simd_float3x3(rows: [
        SIMD3(1,  0,  0),
        SIMD3(0, -1,  0),
        SIMD3(0,  0, -1),
    ])

    private let sessionDirectory: URL
    private let sparseDir: URL

    init(sessionDirectory: URL) {
        self.sessionDirectory = sessionDirectory
        self.sparseDir = sessionDirectory
            .appendingPathComponent("colmap/sparse/0")
    }

    // MARK: - Public

    /// Writes cameras.bin, images.bin, points3D.bin, rigs.bin, and frames.bin.
    func export(scans: [ScanExportData], cameraConfig: MultiCameraConfig) throws {
        try FileManager.default.createDirectory(
            at: sparseDir, withIntermediateDirectories: true)

        guard let first = scans.first else { return }

        let tileSize = 1024
        let tileFovDeg: Float = 65.0
        let f_tile = Float(tileSize) / (2.0 * tan(tileFovDeg * .pi / 360.0))
        let c_tile = Float(tileSize) / 2.0

        // Camera IDs: 1 = iPhone, 2+ = one per active face
        let iphoneCameraId: UInt32 = 1
        // We write one camera per face direction (8 faces)
        let faceCount = 8
        let tileCameraIds: [Int: UInt32] = Dictionary(
            uniqueKeysWithValues: (0..<faceCount).map { ($0, UInt32($0 + 2)) }
        )

        let imgW = first.imageWidth
        let imgH = first.imageHeight
        let fx = first.intrinsics[0][0]
        let fy = first.intrinsics[1][1]
        let cx = first.intrinsics[0][2]
        let cy = first.intrinsics[1][2]

        try writeCamerasBin(
            iphoneCameraId: iphoneCameraId,
            imgW: imgW, imgH: imgH,
            fx: fx, fy: fy, cx: cx, cy: cy,
            tileCameraIds: tileCameraIds,
            tileSize: tileSize,
            f_tile: f_tile, c_tile: c_tile
        )

        let (imageEntries, allPoints) = buildImageEntriesAndPoints(
            scans: scans,
            cameraConfig: cameraConfig,
            iphoneCameraId: iphoneCameraId,
            tileCameraIds: tileCameraIds,
            imgW: imgW, imgH: imgH,
            fx: fx, fy: fy, cx: cx, cy: cy
        )

        try writeImagesBin(entries: imageEntries)
        try writePoints3DBin(points: allPoints)
        try writeRigsBin(cameraConfig: cameraConfig, tileCameraIds: tileCameraIds,
                         iphoneCameraId: iphoneCameraId)
        try writeFramesBin(scans: scans, imageEntries: imageEntries,
                           cameraConfig: cameraConfig, iphoneCameraId: iphoneCameraId,
                           tileCameraIds: tileCameraIds)
    }

    // MARK: - cameras.bin

    private func writeCamerasBin(
        iphoneCameraId: UInt32,
        imgW: Int, imgH: Int,
        fx: Float, fy: Float, cx: Float, cy: Float,
        tileCameraIds: [Int: UInt32],
        tileSize: Int,
        f_tile: Float, c_tile: Float
    ) throws {
        var data = Data()
        let total = UInt64(1 + tileCameraIds.count)
        data.appendLE(total)

        // iPhone — PINHOLE: fx, fy, cx, cy
        data.appendLE(iphoneCameraId)
        data.appendLE(CameraModel.pinhole.rawValue)
        data.appendLE(UInt64(imgW))
        data.appendLE(UInt64(imgH))
        for v in [fx, fy, cx, cy] { data.appendLE(Double(v)) }

        // Face tiles — SIMPLE_PINHOLE: f, cx, cy
        for (_, camId) in tileCameraIds.sorted(by: { $0.key < $1.key }) {
            data.appendLE(camId)
            data.appendLE(CameraModel.simplePinhole.rawValue)
            data.appendLE(UInt64(tileSize))
            data.appendLE(UInt64(tileSize))
            for v in [f_tile, c_tile, c_tile] { data.appendLE(Double(v)) }
        }

        try data.write(to: sparseDir.appendingPathComponent("cameras.bin"))
    }

    // MARK: - images.bin + points

    private func buildImageEntriesAndPoints(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig,
        iphoneCameraId: UInt32,
        tileCameraIds: [Int: UInt32],
        imgW: Int, imgH: Int,
        fx: Float, fy: Float, cx: Float, cy: Float
    ) -> (entries: [ImageEntry], points: [SIMD3<Float>]) {
        var entries: [ImageEntry] = []
        var allPoints: [SIMD3<Float>] = []
        var imageId: UInt32 = 1

        let faceRotations = Self.computeFaceRotations()

        for scan in scans {
            let T_wc = scan.arkitPose  // ARKit camera-to-world (4x4)
            let (R_w2c, t_w2c) = Self.arkitToColmapW2C(T_wc)

            // iPhone image entry
            entries.append(ImageEntry(
                imageId: imageId,
                quatWXYZ: Self.rotationToQuatWXYZ(R_w2c),
                tvec: SIMD3(t_w2c.columns.3.x, t_w2c.columns.3.y, t_w2c.columns.3.z),
                cameraId: iphoneCameraId,
                name: "face_iphone/\(scan.scanName).jpg"
            ))
            imageId += 1

            // Insta360 face tile entries
            for camCfg in cameraConfig.cameras {
                let T_insta_iphone = camCfg.extrinsic.toMatrix()
                // T_iphone_world = inv(T_wc) = world-to-iphone
                let T_iphone_world = T_wc.inverse
                let T_insta_world = T_insta_iphone * T_iphone_world  // insta360 w2c

                let R_insta_w2c = simd_float3x3(
                    SIMD3(T_insta_world.columns.0.x, T_insta_world.columns.0.y, T_insta_world.columns.0.z),
                    SIMD3(T_insta_world.columns.1.x, T_insta_world.columns.1.y, T_insta_world.columns.1.z),
                    SIMD3(T_insta_world.columns.2.x, T_insta_world.columns.2.y, T_insta_world.columns.2.z)
                )
                let t_insta = SIMD3(T_insta_world.columns.3.x,
                                    T_insta_world.columns.3.y,
                                    T_insta_world.columns.3.z)
                let C_insta = -R_insta_w2c.transpose * t_insta
                let C_insta_col = Self.arkitToColmap * C_insta
                let R_insta_col = Self.arkitToColmap * R_insta_w2c

                for (faceIdx, camFromPano) in faceRotations.enumerated() {
                    guard let camId = tileCameraIds[faceIdx] else { continue }
                    let R_tile = camFromPano * R_insta_col
                    let t_tile = -(R_tile * C_insta_col)
                    entries.append(ImageEntry(
                        imageId: imageId,
                        quatWXYZ: Self.rotationToQuatWXYZ(R_tile),
                        tvec: t_tile,
                        cameraId: camId,
                        name: "face_\(String(format: "%02d", faceIdx))/\(scan.scanName).jpg"
                    ))
                    imageId += 1
                }
            }

            // LiDAR depth unprojection
            if let depthPts = unprojectDepth(
                scan: scan, T_wc: T_wc,
                imgW: imgW, imgH: imgH,
                fx: fx, fy: fy, cx: cx, cy: cy
            ) {
                allPoints.append(contentsOf: depthPts)
            }
        }

        return (entries, allPoints)
    }

    // MARK: - LiDAR unprojection

    private func unprojectDepth(
        scan: ScanExportData,
        T_wc: simd_float4x4,
        imgW: Int, imgH: Int,
        fx: Float, fy: Float, cx: Float, cy: Float
    ) -> [SIMD3<Float>]? {
        guard let depthURL = scan.depthBinURL else { return nil }
        guard let rawData = try? Data(contentsOf: depthURL) else { return nil }

        let lidarW = 256, lidarH = 192
        let count = lidarW * lidarH
        guard rawData.count >= count * 4 else { return nil }

        let depths = rawData.withUnsafeBytes { ptr -> [Float] in
            Array(ptr.bindMemory(to: Float.self).prefix(count))
        }

        // Scale intrinsics to LiDAR resolution
        let scaleX = Float(lidarW) / Float(imgW)
        let scaleY = Float(lidarH) / Float(imgH)
        let fx_l = fx * scaleX, fy_l = fy * scaleY
        let cx_l = cx * scaleX, cy_l = cy * scaleY

        var pts: [SIMD3<Float>] = []
        pts.reserveCapacity(count / 4)

        for row in 0..<lidarH {
            for col in 0..<lidarW {
                let z = depths[row * lidarW + col]
                guard z > 0.1 && z < 5.0 else { continue }
                let x_cam = (Float(col) - cx_l) / fx_l * z
                let y_cam = (Float(row) - cy_l) / fy_l * z
                let p_cam = SIMD4<Float>(x_cam, y_cam, z, 1)
                let p_arkit = T_wc * p_cam
                let p_col = Self.arkitToColmap * SIMD3(p_arkit.x, p_arkit.y, p_arkit.z)
                pts.append(p_col)
            }
        }
        return pts
    }

    // MARK: - images.bin

    private func writeImagesBin(entries: [ImageEntry]) throws {
        var data = Data()
        data.appendLE(UInt64(entries.count))
        for e in entries {
            data.appendLE(e.imageId)
            for v in e.quatWXYZ { data.appendLE(Double(v)) }
            data.appendLE(Double(e.tvec.x))
            data.appendLE(Double(e.tvec.y))
            data.appendLE(Double(e.tvec.z))
            data.appendLE(e.cameraId)
            data.append(e.name.data(using: .utf8)!)
            data.append(0)       // null terminator
            data.appendLE(UInt64(0))  // no 2D points
        }
        try data.write(to: sparseDir.appendingPathComponent("images.bin"))
    }

    // MARK: - points3D.bin

    private func writePoints3DBin(points: [SIMD3<Float>]) throws {
        var data = Data()
        data.appendLE(UInt64(points.count))
        for (i, p) in points.enumerated() {
            data.appendLE(UInt64(i + 1))
            data.appendLE(Double(p.x))
            data.appendLE(Double(p.y))
            data.appendLE(Double(p.z))
            data.append(contentsOf: [UInt8(128), 128, 128])  // grey
            data.appendLE(Double(0))   // error
            data.appendLE(UInt64(0))   // track_len = 0
        }
        try data.write(to: sparseDir.appendingPathComponent("points3D.bin"))
    }

    // MARK: - rigs.bin

    /// Writes rigs.bin: one rig with iPhone as ref sensor + one non-ref per Insta360 camera.
    /// Sensor type 0 = camera (COLMAP convention).
    /// Mirrors panorama_sfm_colmap.py write_rig_bin().
    private func writeRigsBin(
        cameraConfig: MultiCameraConfig,
        tileCameraIds: [Int: UInt32],
        iphoneCameraId: UInt32
    ) throws {
        guard !cameraConfig.cameras.isEmpty else { return }

        // Rig ID = 1, ref sensor = iPhone (camera_id 1)
        let rigId: UInt32 = 1
        let sensorTypeCamera: Int32 = 0
        let faceRotations = Self.computeFaceRotations()
        let numSensors = UInt32(1 + cameraConfig.cameras.count * tileCameraIds.count)

        var data = Data()
        data.appendLE(UInt64(1))          // num_rigs
        data.appendLE(rigId)
        data.appendLE(numSensors)
        // Ref sensor: iPhone
        data.appendLE(sensorTypeCamera)
        data.appendLE(iphoneCameraId)

        // Non-ref sensors: each Insta360 face tile
        for camCfg in cameraConfig.cameras {
            let T_insta_iphone = camCfg.extrinsic.toMatrix()
            let R_insta = simd_float3x3(
                SIMD3(T_insta_iphone.columns.0.x, T_insta_iphone.columns.0.y, T_insta_iphone.columns.0.z),
                SIMD3(T_insta_iphone.columns.1.x, T_insta_iphone.columns.1.y, T_insta_iphone.columns.1.z),
                SIMD3(T_insta_iphone.columns.2.x, T_insta_iphone.columns.2.y, T_insta_iphone.columns.2.z)
            )
            let t_insta = SIMD3(T_insta_iphone.columns.3.x,
                                T_insta_iphone.columns.3.y,
                                T_insta_iphone.columns.3.z)

            for (faceIdx, camId) in tileCameraIds.sorted(by: { $0.key < $1.key }) {
                let camFromPano = faceRotations[faceIdx]
                // T_tile_from_iphone = R_tile @ T_insta_iphone
                let R_tile = camFromPano * R_insta
                let t_tile = camFromPano * t_insta
                let q = Self.rotationToQuatWXYZ(R_tile)  // wxyz

                data.appendLE(sensorTypeCamera)
                data.appendLE(camId)
                data.appendLE(UInt8(1))   // has_pose = true
                // sensor_from_rig: qw, qx, qy, qz, tx, ty, tz (7 x float64)
                for v in q { data.appendLE(Double(v)) }
                data.appendLE(Double(t_tile.x))
                data.appendLE(Double(t_tile.y))
                data.appendLE(Double(t_tile.z))
            }
        }

        try data.write(to: sparseDir.appendingPathComponent("rigs.bin"))
    }

    // MARK: - frames.bin

    /// Writes frames.bin: one frame per scan, mapping each scan to its image IDs.
    /// Frame pose = iPhone image w2c (the rig reference sensor).
    private func writeFramesBin(
        scans: [ScanExportData],
        imageEntries: [ImageEntry],
        cameraConfig: MultiCameraConfig,
        iphoneCameraId: UInt32,
        tileCameraIds: [Int: UInt32]
    ) throws {
        let rigId: UInt32 = 1
        let sensorTypeCamera: Int32 = 0
        // Images per scan: 1 iPhone + N_cameras * 8 faces
        let imagesPerScan = 1 + cameraConfig.cameras.count * tileCameraIds.count

        var data = Data()
        data.appendLE(UInt64(scans.count))  // num_frames

        for (scanIdx, scan) in scans.enumerated() {
            let baseImageId = UInt32(scanIdx * imagesPerScan + 1)
            let iphoneImageId = baseImageId

            // Frame pose = iPhone w2c
            let (R_w2c, t_w2c_mat) = Self.arkitToColmapW2C(scan.arkitPose)
            let t_w2c = SIMD3(t_w2c_mat.columns.3.x,
                               t_w2c_mat.columns.3.y,
                               t_w2c_mat.columns.3.z)
            let q = Self.rotationToQuatWXYZ(R_w2c)

            data.appendLE(UInt32(scanIdx + 1))  // frame_id
            data.appendLE(rigId)
            // rig_from_world pose: qw qx qy qz tx ty tz
            for v in q { data.appendLE(Double(v)) }
            data.appendLE(Double(t_w2c.x))
            data.appendLE(Double(t_w2c.y))
            data.appendLE(Double(t_w2c.z))

            // num_data entries
            data.appendLE(UInt32(imagesPerScan))

            // iPhone entry
            data.appendLE(sensorTypeCamera)
            data.appendLE(iphoneCameraId)
            data.appendLE(UInt64(iphoneImageId))

            // Tile entries
            var tileImageId = baseImageId + 1
            for _ in cameraConfig.cameras {
                for (_, camId) in tileCameraIds.sorted(by: { $0.key < $1.key }) {
                    data.appendLE(sensorTypeCamera)
                    data.appendLE(camId)
                    data.appendLE(UInt64(tileImageId))
                    tileImageId += 1
                }
            }
        }

        try data.write(to: sparseDir.appendingPathComponent("frames.bin"))
    }

    // MARK: - Coordinate math

    /// ARKit camera-to-world 4x4 → COLMAP world-to-camera (R_w2c 3x3, t as 4x4 col)
    static func arkitToColmapW2C(_ T_wc: simd_float4x4) -> (simd_float3x3, simd_float4x4) {
        let R_c2w_arkit = simd_float3x3(
            SIMD3(T_wc.columns.0.x, T_wc.columns.0.y, T_wc.columns.0.z),
            SIMD3(T_wc.columns.1.x, T_wc.columns.1.y, T_wc.columns.1.z),
            SIMD3(T_wc.columns.2.x, T_wc.columns.2.y, T_wc.columns.2.z)
        )
        let C_arkit = SIMD3(T_wc.columns.3.x, T_wc.columns.3.y, T_wc.columns.3.z)
        let R_c2w_col = arkitToColmap * R_c2w_arkit
        let C_col = arkitToColmap * C_arkit
        let R_w2c = R_c2w_col.transpose
        let t_w2c = -(R_w2c * C_col)
        var result = simd_float4x4(1)
        result.columns.3 = SIMD4(t_w2c, 1)
        return (R_w2c, result)
    }

    static func rotationToQuatWXYZ(_ R: simd_float3x3) -> [Float] {
        let q = simd_quatf(R)
        let sign: Float = q.real >= 0 ? 1 : -1
        return [q.real * sign, q.imag.x * sign, q.imag.y * sign, q.imag.z * sign]
    }

    /// 8 equatorial face rotations matching atlas panorama_sfm_colmap.py FACES_CAM_FROM_PANO.
    static func computeFaceRotations() -> [simd_float3x3] {
        let instaCorrection = simd_quatf(angle: .pi / 2, axis: SIMD3(0, 1, 0))
        return (0..<8).map { i in
            let yawDeg = Float(i) * 45.0
            let yawRad = yawDeg * .pi / 180.0
            let faceRot = simd_quatf(angle: -yawRad, axis: SIMD3(0, 1, 0))
            return simd_float3x3(faceRot * instaCorrection)
        }
    }
}

// MARK: - Supporting types

struct ScanExportData {
    let scanName: String
    let arkitPose: simd_float4x4       // ARKit camera-to-world
    let intrinsics: [[Float]]          // 3x3 from pose.json
    let imageWidth: Int
    let imageHeight: Int
    let depthBinURL: URL?
    let iphoneImageURL: URL?
    let insta360ImageURLs: [String: URL]  // cameraId → ERP path
}

private struct ImageEntry {
    let imageId: UInt32
    let quatWXYZ: [Float]
    let tvec: SIMD3<Float>
    let cameraId: UInt32
    let name: String
}

// MARK: - Data helpers (already defined in COLMAPExporter originally, keep here)

private extension Data {
    mutating func appendLE<T: FixedWidthInteger>(_ value: T) {
        var v = value.littleEndian
        append(Data(bytes: &v, count: MemoryLayout<T>.size))
    }
    mutating func appendLE(_ value: Double) {
        var v = value
        append(Data(bytes: &v, count: 8))
    }
    mutating func appendLE(_ value: Float) {
        appendLE(Double(value))
    }
    mutating func appendLE(_ value: Int32) {
        var v = value.littleEndian
        append(Data(bytes: &v, count: 4))
    }
}
