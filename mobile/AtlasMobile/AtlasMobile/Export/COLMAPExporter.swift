import Foundation
import simd
import ARKit
import UIKit

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

    // Landscape COLMAP camera frame → portrait COLMAP camera frame.
    // Applied because the saved iPhone JPEG is rotated 90° CW from
    // the landscape sensor buffer.
    static let portraitRotation = simd_float3x3(rows: [
        SIMD3( 0, -1, 0),
        SIMD3( 1,  0, 0),
        SIMD3( 0,  0, 1),
    ])

    // Optimizer Camera frame (X=up,Y=right,Z=forward) from
    // ARKit camera frame (X=right,Y=up,Z=backward).
    private static let cameraFromARKit = simd_float3x3(rows: [
        SIMD3<Float>(0, 1, 0),
        SIMD3<Float>(1, 0, 0),
        SIMD3<Float>(0, 0, -1),
    ])

    /// Seed extrinsic → ERP-from-ARKit 4x4 for composing with T_wc.inverse.
    static func erpFromARKit(_ ext: RigidTransform) -> simd_float4x4 {
        let T = ExtrinsicOptimizer.seedToCameraMatrix(ext)
        let R = simd_float3x3(
            SIMD3(T.columns.0.x, T.columns.0.y, T.columns.0.z),
            SIMD3(T.columns.1.x, T.columns.1.y, T.columns.1.z),
            SIMD3(T.columns.2.x, T.columns.2.y, T.columns.2.z)
        )
        let Rc = R * cameraFromARKit
        var M = simd_float4x4(1)
        M.columns.0 = SIMD4(Rc.columns.0, 0)
        M.columns.1 = SIMD4(Rc.columns.1, 0)
        M.columns.2 = SIMD4(Rc.columns.2, 0)
        M.columns.3 = T.columns.3
        return M
    }

    private let sessionDirectory: URL
    private let sparseDir: URL
    private let imagesDir: URL
    private let depthsDir: URL

    init(sessionDirectory: URL) {
        self.sessionDirectory = sessionDirectory
        self.sparseDir = sessionDirectory
            .appendingPathComponent("colmap/sparse/0")
        self.imagesDir = sessionDirectory
            .appendingPathComponent("colmap/images")
        self.depthsDir = sessionDirectory
            .appendingPathComponent("colmap/depths")
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

        let iphoneCameraId: UInt32 = 1
        let faceCount = 8
        let hasInsta360 = scans.contains { !$0.insta360ImageURLs.isEmpty }
        let tileCameraIds: [Int: UInt32] = hasInsta360
            ? Dictionary(uniqueKeysWithValues: (0..<faceCount).map { ($0, UInt32($0 + 2)) })
            : [:]

        // Landscape values from pose.json (sensor native frame).
        let imgW_l = first.imageWidth
        let imgH_l = first.imageHeight
        let fx_l = first.intrinsics[0][0]
        let fy_l = first.intrinsics[1][1]
        let cx_l = first.intrinsics[0][2]
        let cy_l = first.intrinsics[1][2]

        // Portrait values for cameras.bin — saved JPEG is rotated 90° CW.
        let imgW_p = imgH_l
        let imgH_p = imgW_l
        let fx_p = fy_l
        let fy_p = fx_l
        let cx_p = Float(imgH_l - 1) - cy_l
        let cy_p = cx_l

        try writeCamerasBin(
            iphoneCameraId: iphoneCameraId,
            imgW: imgW_p, imgH: imgH_p,
            fx: fx_p, fy: fy_p, cx: cx_p, cy: cy_p,
            tileCameraIds: tileCameraIds,
            tileSize: tileSize,
            f_tile: f_tile, c_tile: c_tile
        )

        let (imageEntries, allPoints) = buildImageEntriesAndPoints(
            scans: scans,
            cameraConfig: cameraConfig,
            iphoneCameraId: iphoneCameraId,
            tileCameraIds: tileCameraIds,
            imgW: imgW_l, imgH: imgH_l,
            fx: fx_l, fy: fy_l, cx: cx_l, cy: cy_l
        )

        try writeImagesBin(entries: imageEntries)
        try writePoints3DBin(points: allPoints)

        if hasInsta360 {
            try writeRigsBin(
                cameraConfig: cameraConfig,
                tileCameraIds: tileCameraIds,
                iphoneCameraId: iphoneCameraId
            )
            try writeFramesBin(
                scans: scans,
                imageEntries: imageEntries,
                cameraConfig: cameraConfig,
                iphoneCameraId: iphoneCameraId,
                tileCameraIds: tileCameraIds
            )
        }
        try copyImagesAndDepths(scans: scans)
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
            let (R_w2c_landscape, t_w2c_mat) = Self.arkitToColmapW2C(T_wc)
            let t_w2c_landscape = SIMD3(
                t_w2c_mat.columns.3.x,
                t_w2c_mat.columns.3.y,
                t_w2c_mat.columns.3.z
            )

            // iPhone image entry — portrait rotation (saved JPEG is 90° CW)
            let R_w2c_portrait = Self.portraitRotation * R_w2c_landscape
            let t_w2c_portrait = Self.portraitRotation * t_w2c_landscape

            entries.append(ImageEntry(
                imageId: imageId,
                quatWXYZ: Self.rotationToQuatWXYZ(R_w2c_portrait),
                tvec: t_w2c_portrait,
                cameraId: iphoneCameraId,
                name: "face_iphone/\(scan.scanName).jpg"
            ))
            imageId += 1

            // Insta360 face tile entries — only for scans that have ERP images
            if !scan.insta360ImageURLs.isEmpty {
                for camCfg in cameraConfig.cameras {
                    let T_insta_arkit = Self.erpFromARKit(camCfg.extrinsic)
                    let T_iphone_world = T_wc.inverse
                    let T_insta_world = T_insta_arkit * T_iphone_world

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
                    let R_insta_col = R_insta_w2c * Self.arkitToColmap

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

        let step = 4
        var pts: [SIMD3<Float>] = []
        pts.reserveCapacity((lidarH / step) * (lidarW / step))

        for row in Swift.stride(from: 0, to: lidarH, by: step) {
            for col in Swift.stride(from: 0, to: lidarW, by: step) {
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
        let url = sparseDir.appendingPathComponent("points3D.bin")
        FileManager.default.createFile(atPath: url.path, contents: nil)
        let handle = try FileHandle(forWritingTo: url)
        defer { try? handle.close() }

        var header = Data()
        header.appendLE(UInt64(points.count))
        handle.write(header)

        let batchSize = 1024
        for batchStart in Swift.stride(
            from: 0, to: points.count, by: batchSize
        ) {
            var chunk = Data(capacity: batchSize * 51)
            let end = min(batchStart + batchSize, points.count)
            for i in batchStart..<end {
                let p = points[i]
                chunk.appendLE(UInt64(i + 1))
                chunk.appendLE(Double(p.x))
                chunk.appendLE(Double(p.y))
                chunk.appendLE(Double(p.z))
                chunk.append(contentsOf: [UInt8(128), 128, 128])
                chunk.appendLE(Double(0))
                chunk.appendLE(UInt64(0))
            }
            handle.write(chunk)
        }
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

        // Non-ref sensors: each Insta360 face tile.
        // sensor_from_rig = cam_from_pano * erp_from_arkit * portraitRot^T
        let rRotT = Self.portraitRotation.transpose
        for camCfg in cameraConfig.cameras {
            let T_erp = Self.erpFromARKit(camCfg.extrinsic)
            let R_insta = simd_float3x3(
                SIMD3(T_erp.columns.0.x, T_erp.columns.0.y, T_erp.columns.0.z),
                SIMD3(T_erp.columns.1.x, T_erp.columns.1.y, T_erp.columns.1.z),
                SIMD3(T_erp.columns.2.x, T_erp.columns.2.y, T_erp.columns.2.z)
            )
            let t_insta = SIMD3(T_erp.columns.3.x,
                                T_erp.columns.3.y,
                                T_erp.columns.3.z)

            for (faceIdx, camId) in tileCameraIds.sorted(by: { $0.key < $1.key }) {
                let camFromPano = faceRotations[faceIdx]
                let R_tile = camFromPano * R_insta * rRotT
                let t_tile = camFromPano * t_insta
                let q = Self.rotationToQuatWXYZ(R_tile)

                data.appendLE(sensorTypeCamera)
                data.appendLE(camId)
                data.appendLE(UInt8(1))   // has_pose = true
                for v in q { data.appendLE(Double(v)) }
                data.appendLE(Double(t_tile.x))
                data.appendLE(Double(t_tile.y))
                data.appendLE(Double(t_tile.z))
            }
        }

        try data.write(to: sparseDir.appendingPathComponent("rigs.bin"))
    }

    // MARK: - frames.bin

    /// Writes frames.bin: one frame per rig scan (scans with Insta360 images).
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
        let sensorsPerFrame = 1 + cameraConfig.cameras.count * tileCameraIds.count

        // Index image entries by name for O(1) lookup
        let entryByName = Dictionary(
            uniqueKeysWithValues: imageEntries.map { ($0.name, $0) }
        )

        // Only include scans that have all rig cameras
        let rigScans = scans.filter { !$0.insta360ImageURLs.isEmpty }

        var data = Data()
        data.appendLE(UInt64(rigScans.count))

        for (frameIdx, scan) in rigScans.enumerated() {
            let (R_w2c_l, t_w2c_mat) = Self.arkitToColmapW2C(scan.arkitPose)
            let t_w2c_l = SIMD3(
                t_w2c_mat.columns.3.x,
                t_w2c_mat.columns.3.y,
                t_w2c_mat.columns.3.z
            )
            let R_w2c = Self.portraitRotation * R_w2c_l
            let t_w2c = Self.portraitRotation * t_w2c_l
            let q = Self.rotationToQuatWXYZ(R_w2c)

            data.appendLE(UInt32(frameIdx + 1))
            data.appendLE(rigId)
            for v in q { data.appendLE(Double(v)) }
            data.appendLE(Double(t_w2c.x))
            data.appendLE(Double(t_w2c.y))
            data.appendLE(Double(t_w2c.z))

            data.appendLE(UInt32(sensorsPerFrame))

            // iPhone entry
            let iphoneName = "face_iphone/\(scan.scanName).jpg"
            let iphoneEntry = entryByName[iphoneName]
            data.appendLE(sensorTypeCamera)
            data.appendLE(iphoneCameraId)
            data.appendLE(UInt64(iphoneEntry?.imageId ?? 0))

            // Tile entries
            for _ in cameraConfig.cameras {
                for (faceIdx, camId) in tileCameraIds.sorted(by: { $0.key < $1.key }) {
                    let tileName = "face_\(String(format: "%02d", faceIdx))/\(scan.scanName).jpg"
                    let tileEntry = entryByName[tileName]
                    data.appendLE(sensorTypeCamera)
                    data.appendLE(camId)
                    data.appendLE(UInt64(tileEntry?.imageId ?? 0))
                }
            }
        }

        try data.write(to: sparseDir.appendingPathComponent("frames.bin"))
    }

    // MARK: - Copy images & depths

    private func copyImagesAndDepths(
        scans: [ScanExportData]
    ) throws {
        let fm = FileManager.default
        let iphoneFaceDir = imagesDir.appendingPathComponent("face_iphone")
        let masksDir = sessionDirectory.appendingPathComponent("colmap/masks")
        let iphoneMaskDir = masksDir.appendingPathComponent("face_iphone")
        try fm.createDirectory(at: iphoneFaceDir, withIntermediateDirectories: true)
        try fm.createDirectory(at: depthsDir, withIntermediateDirectories: true)
        try fm.createDirectory(at: masksDir, withIntermediateDirectories: true)
        try fm.createDirectory(at: iphoneMaskDir, withIntermediateDirectories: true)

        let slicer = ERPTileSlicer()
        let colorProfile = ColorProfileCalibrator.load()
        if let cp = colorProfile {
            print("[Export] color profile: L×\(cp.lGain) a\(cp.aOffset) b\(cp.bOffset)")
        } else {
            print("[Export] no color profile — exporting uncorrected")
        }

        let maskImages = loadAllCameraMasks()

        for scan in scans { autoreleasepool {
            // iPhone RGB
            if let src = scan.iphoneImageURL, fm.fileExists(atPath: src.path) {
                let dst = iphoneFaceDir.appendingPathComponent(
                    "\(scan.scanName).jpg"
                )
                try? fm.copyItem(at: src, to: dst)

                let maskDst = iphoneMaskDir.appendingPathComponent(
                    "\(scan.scanName).png"
                )
                Self.writeWhiteMask(
                    to: maskDst, width: scan.imageHeight,
                    height: scan.imageWidth
                )
            }

            // Smoothed depth
            if let src = scan.smoothedDepthBinURL, fm.fileExists(atPath: src.path) {
                let dst = depthsDir.appendingPathComponent(
                    "\(scan.scanName)_depth_smoothed.bin"
                )
                try? fm.copyItem(at: src, to: dst)
            }

            // Insta360 ERP → perspective face tiles
            for (camId, erpURL) in scan.insta360ImageURLs {
                guard let erpData = try? Data(contentsOf: erpURL),
                      let rawImage = UIImage(data: erpData)
                else { continue }

                let stitchedERP: UIImage
                if DualFisheyeStitcher.isDualFisheye(rawImage) {
                    guard let stitched = DualFisheyeStitcher.stitch(
                        rawImage, outputWidth: 2048
                    ) else { continue }
                    stitchedERP = stitched
                } else {
                    stitchedERP = rawImage
                }

                let erpImage: UIImage
                if let cp = colorProfile {
                    erpImage = ColorProfileCalibrator.apply(
                        cp, to: stitchedERP
                    )
                } else {
                    erpImage = stitchedERP
                }

                let tiles = slicer.sliceTiles(from: erpImage)
                for (faceIdx, tileImage) in tiles {
                    let faceDir = imagesDir.appendingPathComponent(
                        String(format: "face_%02d", faceIdx)
                    )
                    try? fm.createDirectory(
                        at: faceDir, withIntermediateDirectories: true
                    )
                    let dst = faceDir.appendingPathComponent(
                        "\(scan.scanName).jpg"
                    )
                    if let jpegData = tileImage.jpegData(
                        compressionQuality: 0.92
                    ) {
                        try? jpegData.write(to: dst)
                    }
                }

                if let erpMask = maskImages[camId] {
                    let invertedMask = Self.invertMask(erpMask)
                    let maskTiles = slicer.sliceMask(
                        from: invertedMask
                    )
                    for (faceIdx, maskTile) in maskTiles {
                        let maskFaceDir = masksDir.appendingPathComponent(
                            String(format: "face_%02d", faceIdx)
                        )
                        try? fm.createDirectory(
                            at: maskFaceDir,
                            withIntermediateDirectories: true
                        )
                        let dst = maskFaceDir.appendingPathComponent(
                            "\(scan.scanName).png"
                        )
                        if let pngData = maskTile.pngData() {
                            try? pngData.write(to: dst)
                        }
                    }
                } else {
                    let tileSize = slicer.tileSize
                    for faceIdx in 0..<8 {
                        let maskFaceDir = masksDir.appendingPathComponent(
                            String(format: "face_%02d", faceIdx)
                        )
                        try? fm.createDirectory(
                            at: maskFaceDir,
                            withIntermediateDirectories: true
                        )
                        let dst = maskFaceDir.appendingPathComponent(
                            "\(scan.scanName).png"
                        )
                        Self.writeWhiteMask(
                            to: dst, width: tileSize, height: tileSize
                        )
                    }
                }
            }
        } }
    }

    /// All-white mask (COLMAP: white=valid).
    private static func writeWhiteMask(
        to url: URL, width: Int, height: Int
    ) {
        let renderer = UIGraphicsImageRenderer(
            size: CGSize(width: width, height: height)
        )
        let img = renderer.image { _ in
            UIColor.white.setFill()
            UIRectFill(CGRect(x: 0, y: 0, width: width, height: height))
        }
        if let data = img.pngData() {
            try? data.write(to: url)
        }
    }

    /// Inverts a mask image (drawn: white=masked → COLMAP: white=valid).
    private static func invertMask(_ mask: UIImage) -> UIImage {
        guard let cg = mask.cgImage else { return mask }
        let w = cg.width, h = cg.height
        var pixels = [UInt8](repeating: 0, count: w * h * 4)
        guard let ctx = CGContext(
            data: &pixels, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return mask }
        ctx.draw(cg, in: CGRect(x: 0, y: 0, width: w, height: h))
        for i in stride(from: 0, to: pixels.count, by: 4) {
            pixels[i]     = 255 - pixels[i]
            pixels[i + 1] = 255 - pixels[i + 1]
            pixels[i + 2] = 255 - pixels[i + 2]
        }
        guard let outCtx = CGContext(
            data: &pixels, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ), let result = outCtx.makeImage() else { return mask }
        return UIImage(cgImage: result)
    }

    private func loadAllCameraMasks() -> [String: UIImage] {
        guard let dir = FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first?.appendingPathComponent("atlas_sessions/masks")
        else { return [:] }
        var result: [String: UIImage] = [:]
        if let files = try? FileManager.default.contentsOfDirectory(
            at: dir, includingPropertiesForKeys: nil
        ) {
            for file in files where file.pathExtension == "png" {
                let camId = file.deletingPathExtension().lastPathComponent
                if let data = try? Data(contentsOf: file),
                   let img = UIImage(data: data) {
                    result[camId] = img
                }
            }
        }
        return result
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

    /// 8 equatorial face rotations (cam_from_pano) at 45° yaw steps.
    /// Must match ERPTileSlicer.computeFaceRotations exactly.
    static func computeFaceRotations() -> [simd_float3x3] {
        (0..<8).map { i in
            let yawRad = Float(i) * .pi / 4.0
            let faceQuat = simd_quatf(angle: -yawRad, axis: SIMD3(0, 1, 0))
            return simd_float3x3(faceQuat)
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
    let smoothedDepthBinURL: URL?
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
