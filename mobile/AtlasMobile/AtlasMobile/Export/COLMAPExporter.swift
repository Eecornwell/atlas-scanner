import Accelerate
import Foundation
import ImageIO
import simd
import ARKit
import UIKit

/// How to export Insta360 images in the COLMAP model.
enum CameraExportMode: String, CaseIterable {
    /// ERP → 8 perspective tiles (SIMPLE_PINHOLE). Works with all pipelines.
    case pinholeTiles
    /// ERP → 2 equidistant fisheye lens images (OPENCV_FISHEYE). Native FOV, needs 3DGUT.
    case nativeFisheye
}

/// Exports a captured session to COLMAP binary format on-device.
/// Mirrors assemble_colmap.py — produces cameras.bin, images.bin, points3D.bin.
/// ERP tile slicing and depth image generation are deferred to the offline pipeline.
final class COLMAPExporter {

    // COLMAP camera model IDs
    private enum CameraModel: Int32 {
        case simplePinhole = 0  // f, cx, cy — Insta360 face tiles
        case pinhole = 1        // fx, fy, cx, cy — iPhone
        case opencvFisheye = 5  // fx, fy, cx, cy, k1, k2, k3, k4
    }

    // Fisheye lens constants — match DualFisheyeStitcher's equidistant model
    private static let fisheyeSize = 1024
    private static let fisheyeHalfFovRad: Float = 100 * .pi / 180
    private static let fisheyeRadiusFrac: Float = 0.47

    // 180° rotation around Y — front lens → back lens orientation
    private static let R_y_pi = simd_float3x3(rows: [
        SIMD3<Float>(-1,  0,  0),
        SIMD3<Float>( 0,  1,  0),
        SIMD3<Float>( 0,  0, -1),
    ])

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

    static let blurThreshold: Float = 300.0

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
    func export(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig,
        cameraExportMode: CameraExportMode = .pinholeTiles
    ) throws {
        try FileManager.default.createDirectory(
            at: sparseDir, withIntermediateDirectories: true)

        guard let first = scans.first else { return }

        let tileSize = 1024
        let tileFovDeg: Float = 65.0
        let f_tile = Float(tileSize) / (2.0 * tan(tileFovDeg * .pi / 360.0))
        let c_tile = Float(tileSize) / 2.0

        let iphoneCameraId: UInt32 = 1
        let faceCount = 8

        var filteredScans: [ScanExportData] = []
        var skippedNames: [String] = []
        for scan in scans {
            let skipped: Bool = autoreleasepool {
                if let url = scan.iphoneImageURL,
                   let score = Self.blurScore(
                       imageURL: url
                   ),
                   score < Self.blurThreshold
                {
                    let tag = String(
                        format: "%.1f", score
                    )
                    skippedNames.append(
                        "\(scan.scanName) (\(tag))"
                    )
                    return true
                }
                return false
            }
            if !skipped {
                filteredScans.append(scan)
            }
        }
        if filteredScans.isEmpty {
            print(
                "[Export] all scans below blur"
                + " threshold — keeping all"
            )
            filteredScans = scans
        } else if !skippedNames.isEmpty {
            let names = skippedNames.joined(
                separator: ", "
            )
            print(
                "[Export] skipped "
                + "\(skippedNames.count)"
                + " blurry: \(names)"
            )
        }

        let hasInsta360 = filteredScans.contains { !$0.insta360ImageURLs.isEmpty }

        // Landscape values from pose.json (sensor native frame).
        // imageResolution may differ from capturedImage buffer;
        // read the saved JPEG to get actual pixel dimensions.
        var imgW_l = first.imageWidth
        var imgH_l = first.imageHeight
        if let url = first.iphoneImageURL,
           let jpegSize = Self.imageDimensions(url) {
            // JPEG is portrait (rotated .right from landscape buffer)
            let actualW_l = jpegSize.height
            let actualH_l = jpegSize.width
            if actualW_l != first.imageWidth
                || actualH_l != first.imageHeight {
                print(
                    "[Export] imageRes=\(first.imageWidth)"
                    + "×\(first.imageHeight)"
                    + " actual=\(actualW_l)×\(actualH_l)"
                    + " — scaling intrinsics"
                )
                imgW_l = actualW_l
                imgH_l = actualH_l
            }
        }
        let scaleX = Float(imgW_l) / Float(first.imageWidth)
        let scaleY = Float(imgH_l) / Float(first.imageHeight)
        let fx_l = first.intrinsics[0][0] * scaleX
        let fy_l = first.intrinsics[1][1] * scaleY
        let cx_l = first.intrinsics[0][2] * scaleX
        let cy_l = first.intrinsics[1][2] * scaleY

        // Portrait values for cameras.bin — saved JPEG is rotated 90° CW.
        let imgW_p = imgH_l
        let imgH_p = imgW_l
        let fx_p = fy_l
        let fy_p = fx_l
        let cx_p = Float(imgH_l - 1) - cy_l
        let cy_p = cx_l

        let useFisheye = cameraExportMode == .nativeFisheye && hasInsta360

        // Camera IDs for Insta360 sensors
        let tileCameraIds: [Int: UInt32]
        let fisheyeCameraIds: [(front: UInt32, back: UInt32)]

        if useFisheye {
            tileCameraIds = [:]
            fisheyeCameraIds = cameraConfig.cameras.enumerated().map { (i, _) in
                (front: UInt32(2 + i * 2), back: UInt32(3 + i * 2))
            }
            let feSize = Self.fisheyeSize
            let feRadius = Float(feSize) * Self.fisheyeRadiusFrac
            let feFocal = feRadius / Self.fisheyeHalfFovRad
            let feCenter = Float(feSize) / 2.0

            try writeCamerasBin(
                iphoneCameraId: iphoneCameraId,
                imgW: imgW_p, imgH: imgH_p,
                fx: fx_p, fy: fy_p, cx: cx_p, cy: cy_p,
                tileCameraIds: [:],
                tileSize: tileSize,
                f_tile: f_tile, c_tile: c_tile,
                fisheyeCameraIds: fisheyeCameraIds,
                fisheyeSize: feSize,
                fisheyeFocal: feFocal, fisheyeCenter: feCenter
            )
        } else {
            tileCameraIds = hasInsta360
                ? Dictionary(uniqueKeysWithValues: (0..<faceCount).map { ($0, UInt32($0 + 2)) })
                : [:]
            fisheyeCameraIds = []

            try writeCamerasBin(
                iphoneCameraId: iphoneCameraId,
                imgW: imgW_p, imgH: imgH_p,
                fx: fx_p, fy: fy_p, cx: cx_p, cy: cy_p,
                tileCameraIds: tileCameraIds,
                tileSize: tileSize,
                f_tile: f_tile, c_tile: c_tile,
                fisheyeCameraIds: [],
                fisheyeSize: 0, fisheyeFocal: 0, fisheyeCenter: 0
            )
        }

        do {
            let (imageEntries, allPoints) =
                buildImageEntriesAndPoints(
                    scans: filteredScans,
                    cameraConfig: cameraConfig,
                    iphoneCameraId: iphoneCameraId,
                    tileCameraIds: tileCameraIds,
                    fisheyeCameraIds: fisheyeCameraIds,
                    imgW: imgW_l, imgH: imgH_l,
                    fx: fx_l, fy: fy_l,
                    cx: cx_l, cy: cy_l
                )

            try writeImagesBin(entries: imageEntries)
            try writePoints3DBin(points: allPoints)

            if hasInsta360 {
                try writeRigsBin(
                    cameraConfig: cameraConfig,
                    tileCameraIds: tileCameraIds,
                    fisheyeCameraIds: fisheyeCameraIds,
                    iphoneCameraId: iphoneCameraId
                )
                try writeFramesBin(
                    scans: filteredScans,
                    imageEntries: imageEntries,
                    cameraConfig: cameraConfig,
                    iphoneCameraId: iphoneCameraId,
                    tileCameraIds: tileCameraIds,
                    fisheyeCameraIds: fisheyeCameraIds
                )
            }
        }
        try copyImagesAndDepths(
            scans: filteredScans,
            cameraConfig: cameraConfig,
            useFisheye: useFisheye
        )
    }

    // MARK: - cameras.bin

    private func writeCamerasBin(
        iphoneCameraId: UInt32,
        imgW: Int, imgH: Int,
        fx: Float, fy: Float, cx: Float, cy: Float,
        tileCameraIds: [Int: UInt32],
        tileSize: Int,
        f_tile: Float, c_tile: Float,
        fisheyeCameraIds: [(front: UInt32, back: UInt32)],
        fisheyeSize: Int,
        fisheyeFocal: Float, fisheyeCenter: Float
    ) throws {
        var data = Data()
        let total = UInt64(
            1 + tileCameraIds.count + fisheyeCameraIds.count * 2
        )
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

        // Fisheye lenses — OPENCV_FISHEYE: fx, fy, cx, cy, k1, k2, k3, k4
        for ids in fisheyeCameraIds {
            for camId in [ids.front, ids.back] {
                data.appendLE(camId)
                data.appendLE(CameraModel.opencvFisheye.rawValue)
                data.appendLE(UInt64(fisheyeSize))
                data.appendLE(UInt64(fisheyeSize))
                let params: [Float] = [
                    fisheyeFocal, fisheyeFocal,
                    fisheyeCenter, fisheyeCenter,
                    0, 0, 0, 0,
                ]
                for v in params { data.appendLE(Double(v)) }
            }
        }

        try data.write(to: sparseDir.appendingPathComponent("cameras.bin"))
    }

    // MARK: - images.bin + points

    private func buildImageEntriesAndPoints(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig,
        iphoneCameraId: UInt32,
        tileCameraIds: [Int: UInt32],
        fisheyeCameraIds: [(front: UInt32, back: UInt32)],
        imgW: Int, imgH: Int,
        fx: Float, fy: Float, cx: Float, cy: Float
    ) -> (entries: [ImageEntry], points: [ColoredPoint]) {
        var entries: [ImageEntry] = []
        var allPoints: [ColoredPoint] = []
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
                name: "face_iphone/\(scan.scanName).jpg",
                observations: []
            ))
            imageId += 1

            // Insta360 entries — only for scans that have ERP images
            if !scan.insta360ImageURLs.isEmpty {
                for (camIdx, camCfg) in cameraConfig.cameras.enumerated() {
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

                    if !fisheyeCameraIds.isEmpty {
                        // Fisheye mode — 2 images per camera (front + back lens)
                        let ids = fisheyeCameraIds[camIdx]

                        let R_front = R_insta_col
                        let t_front = -(R_front * C_insta_col)
                        entries.append(ImageEntry(
                            imageId: imageId,
                            quatWXYZ: Self.rotationToQuatWXYZ(R_front),
                            tvec: t_front,
                            cameraId: ids.front,
                            name: "fisheye_front/\(scan.scanName).jpg",
                            observations: []
                        ))
                        imageId += 1

                        let R_back = Self.R_y_pi * R_insta_col
                        let t_back = -(R_back * C_insta_col)
                        entries.append(ImageEntry(
                            imageId: imageId,
                            quatWXYZ: Self.rotationToQuatWXYZ(R_back),
                            tvec: t_back,
                            cameraId: ids.back,
                            name: "fisheye_back/\(scan.scanName).jpg",
                            observations: []
                        ))
                        imageId += 1
                    } else {
                        // Tile mode — 8 perspective tiles per camera
                        for (faceIdx, camFromPano) in faceRotations.enumerated() {
                            guard let camId = tileCameraIds[faceIdx] else { continue }
                            let R_tile = camFromPano * R_insta_col
                            let t_tile = -(R_tile * C_insta_col)
                            entries.append(ImageEntry(
                                imageId: imageId,
                                quatWXYZ: Self.rotationToQuatWXYZ(R_tile),
                                tvec: t_tile,
                                cameraId: camId,
                                name: "face_\(String(format: "%02d", faceIdx))/\(scan.scanName).jpg",
                                observations: []
                            ))
                            imageId += 1
                        }
                    }
                }
            }

            autoreleasepool {
                if let depthPts = unprojectDepth(
                    scan: scan, T_wc: T_wc,
                    imgW: imgW, imgH: imgH,
                    fx: fx, fy: fy, cx: cx, cy: cy
                ) {
                    allPoints.append(contentsOf: depthPts)
                }
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
    ) -> [ColoredPoint]? {
        guard let depthURL = scan.depthBinURL else {
            return nil
        }
        guard let rawData = try? Data(
            contentsOf: depthURL
        ) else { return nil }

        let lidarW = 256, lidarH = 192
        let count = lidarW * lidarH
        guard rawData.count >= count * 4 else {
            return nil
        }

        let depths = rawData.withUnsafeBytes {
            ptr -> [Float] in
            Array(
                ptr.bindMemory(to: Float.self)
                    .prefix(count)
            )
        }

        let pixels = Self.loadImageRGBA(
            url: scan.iphoneImageURL
        )

        let scaleX = Float(lidarW) / Float(imgW)
        let scaleY = Float(lidarH) / Float(imgH)
        let fx_l = fx * scaleX, fy_l = fy * scaleY
        let cx_l = cx * scaleX, cy_l = cy * scaleY

        let step = 4
        var pts: [ColoredPoint] = []
        pts.reserveCapacity(
            (lidarH / step) * (lidarW / step)
        )

        for row in Swift.stride(
            from: 0, to: lidarH, by: step
        ) {
            for col in Swift.stride(
                from: 0, to: lidarW, by: step
            ) {
                let z = depths[row * lidarW + col]
                guard z > 0.1 && z < 5.0 else {
                    continue
                }
                let x_cam =
                    (Float(col) - cx_l) / fx_l * z
                let y_cam =
                    (Float(row) - cy_l) / fy_l * z
                let p_cam = SIMD4<Float>(
                    x_cam, -y_cam, -z, 1
                )
                let p_arkit = T_wc * p_cam
                let p_col = Self.arkitToColmap
                    * SIMD3(
                        p_arkit.x,
                        p_arkit.y,
                        p_arkit.z
                    )

                let rgb = Self.sampleColor(
                    pixels: pixels,
                    lidarCol: col, lidarRow: row,
                    imgW: imgW, imgH: imgH,
                    lidarW: lidarW, lidarH: lidarH
                )
                pts.append(ColoredPoint(
                    position: p_col,
                    r: rgb.0, g: rgb.1, b: rgb.2
                ))
            }
        }
        return pts
    }

    private static func loadImageRGBA(
        url: URL?
    ) -> (data: [UInt8], w: Int, h: Int)? {
        autoreleasepool {
            guard let url,
                  let imgData = try? Data(
                      contentsOf: url
                  ),
                  let uiImage = UIImage(data: imgData),
                  let cg = uiImage.cgImage
            else { return nil }
            let w = cg.width, h = cg.height
            guard w > 0, h > 0 else { return nil }
            var buf = [UInt8](
                repeating: 0, count: w * h * 4
            )
            guard let ctx = CGContext(
                data: &buf, width: w, height: h,
                bitsPerComponent: 8,
                bytesPerRow: w * 4,
                space: CGColorSpaceCreateDeviceRGB(),
                bitmapInfo: CGImageAlphaInfo
                    .noneSkipLast.rawValue
            ) else { return nil }
            ctx.draw(
                cg,
                in: CGRect(
                    x: 0, y: 0,
                    width: w, height: h
                )
            )
            return (buf, w, h)
        }
    }

    private static func sampleColor(
        pixels: (data: [UInt8], w: Int, h: Int)?,
        lidarCol: Int, lidarRow: Int,
        imgW: Int, imgH: Int,
        lidarW: Int, lidarH: Int
    ) -> (UInt8, UInt8, UInt8) {
        guard let px = pixels else {
            return (128, 128, 128)
        }
        let lx = Float(lidarCol) * Float(imgW)
            / Float(lidarW)
        let ly = Float(lidarRow) * Float(imgH)
            / Float(lidarH)
        let portX = Int(Float(imgH - 1) - ly)
        let portY = Int(lx)
        guard portX >= 0, portX < px.w,
              portY >= 0, portY < px.h
        else { return (128, 128, 128) }
        let idx = (portY * px.w + portX) * 4
        return (px.data[idx], px.data[idx + 1],
                px.data[idx + 2])
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

    private func writePoints3DBin(
        points: [ColoredPoint]
    ) throws {
        let url = sparseDir.appendingPathComponent(
            "points3D.bin"
        )
        FileManager.default.createFile(
            atPath: url.path, contents: nil
        )
        let handle = try FileHandle(forWritingTo: url)
        defer { try? handle.close() }

        var header = Data()
        header.appendLE(UInt64(points.count))
        handle.write(header)

        let batchSize = 1024
        for batchStart in Swift.stride(
            from: 0, to: points.count, by: batchSize
        ) {
            var chunk = Data(
                capacity: batchSize * 51
            )
            let end = min(
                batchStart + batchSize,
                points.count
            )
            for i in batchStart..<end {
                let p = points[i]
                chunk.appendLE(UInt64(i + 1))
                chunk.appendLE(Double(p.position.x))
                chunk.appendLE(Double(p.position.y))
                chunk.appendLE(Double(p.position.z))
                chunk.append(
                    contentsOf: [p.r, p.g, p.b]
                )
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
        fisheyeCameraIds: [(front: UInt32, back: UInt32)],
        iphoneCameraId: UInt32
    ) throws {
        guard !cameraConfig.cameras.isEmpty else { return }

        let rigId: UInt32 = 1
        let sensorTypeCamera: Int32 = 0
        let faceRotations = Self.computeFaceRotations()
        let rRotT = Self.portraitRotation.transpose

        let nonRefCount: Int
        if !fisheyeCameraIds.isEmpty {
            nonRefCount = cameraConfig.cameras.count * 2
        } else {
            nonRefCount = cameraConfig.cameras.count * tileCameraIds.count
        }
        let numSensors = UInt32(1 + nonRefCount)

        var data = Data()
        data.appendLE(UInt64(1))          // num_rigs
        data.appendLE(rigId)
        data.appendLE(numSensors)
        data.appendLE(sensorTypeCamera)
        data.appendLE(iphoneCameraId)

        for (camIdx, camCfg) in cameraConfig.cameras.enumerated() {
            let T_erp = Self.erpFromARKit(camCfg.extrinsic)
            let R_insta = simd_float3x3(
                SIMD3(T_erp.columns.0.x, T_erp.columns.0.y, T_erp.columns.0.z),
                SIMD3(T_erp.columns.1.x, T_erp.columns.1.y, T_erp.columns.1.z),
                SIMD3(T_erp.columns.2.x, T_erp.columns.2.y, T_erp.columns.2.z)
            )
            let t_insta = SIMD3(T_erp.columns.3.x,
                                T_erp.columns.3.y,
                                T_erp.columns.3.z)

            if !fisheyeCameraIds.isEmpty {
                let ids = fisheyeCameraIds[camIdx]

                // Front lens: identity rotation from ERP → fisheye camera
                let R_front = R_insta * rRotT
                let t_front = t_insta
                let qF = Self.rotationToQuatWXYZ(R_front)
                data.appendLE(sensorTypeCamera)
                data.appendLE(ids.front)
                data.appendLE(UInt8(1))
                for v in qF { data.appendLE(Double(v)) }
                data.appendLE(Double(t_front.x))
                data.appendLE(Double(t_front.y))
                data.appendLE(Double(t_front.z))

                // Back lens: R_y(π) rotation from ERP
                let R_back = Self.R_y_pi * R_insta * rRotT
                let t_back = Self.R_y_pi * t_insta
                let qB = Self.rotationToQuatWXYZ(R_back)
                data.appendLE(sensorTypeCamera)
                data.appendLE(ids.back)
                data.appendLE(UInt8(1))
                for v in qB { data.appendLE(Double(v)) }
                data.appendLE(Double(t_back.x))
                data.appendLE(Double(t_back.y))
                data.appendLE(Double(t_back.z))
            } else {
                for (faceIdx, camId) in tileCameraIds.sorted(by: { $0.key < $1.key }) {
                    let camFromPano = faceRotations[faceIdx]
                    let R_tile = camFromPano * R_insta * rRotT
                    let t_tile = camFromPano * t_insta
                    let q = Self.rotationToQuatWXYZ(R_tile)
                    data.appendLE(sensorTypeCamera)
                    data.appendLE(camId)
                    data.appendLE(UInt8(1))
                    for v in q { data.appendLE(Double(v)) }
                    data.appendLE(Double(t_tile.x))
                    data.appendLE(Double(t_tile.y))
                    data.appendLE(Double(t_tile.z))
                }
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
        tileCameraIds: [Int: UInt32],
        fisheyeCameraIds: [(front: UInt32, back: UInt32)]
    ) throws {
        let rigId: UInt32 = 1
        let sensorTypeCamera: Int32 = 0
        let sensorsPerFrame: Int
        if !fisheyeCameraIds.isEmpty {
            sensorsPerFrame = 1 + cameraConfig.cameras.count * 2
        } else {
            sensorsPerFrame = 1 + cameraConfig.cameras.count * tileCameraIds.count
        }

        let entryByName = Dictionary(
            uniqueKeysWithValues: imageEntries.map { ($0.name, $0) }
        )

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

            if !fisheyeCameraIds.isEmpty {
                // Fisheye entries
                for ids in fisheyeCameraIds {
                    let frontName = "fisheye_front/\(scan.scanName).jpg"
                    let frontEntry = entryByName[frontName]
                    data.appendLE(sensorTypeCamera)
                    data.appendLE(ids.front)
                    data.appendLE(UInt64(frontEntry?.imageId ?? 0))

                    let backName = "fisheye_back/\(scan.scanName).jpg"
                    let backEntry = entryByName[backName]
                    data.appendLE(sensorTypeCamera)
                    data.appendLE(ids.back)
                    data.appendLE(UInt64(backEntry?.imageId ?? 0))
                }
            } else {
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
        }

        try data.write(to: sparseDir.appendingPathComponent("frames.bin"))
    }

    // MARK: - Copy images & depths

    private func copyImagesAndDepths(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig,
        useFisheye: Bool
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

        let invertedMasks: [String: UIImage] =
            loadAllCameraMasks().mapValues {
                Self.invertMask($0)
            }

        var precomputedMaskTiles: [String: [Int: Data]] = [:]
        for (camId, mask) in invertedMasks {
            var tiles: [Int: Data] = [:]
            if useFisheye {
                let feSize = Self.fisheyeSize
                for isFront in [true, false] {
                    autoreleasepool {
                        guard let (_, feMask) =
                            Self.reprojectERPtoFisheye(
                                erpImage: mask,
                                erpMask: nil,
                                fisheyeSize: feSize,
                                isFrontLens: isFront
                            ),
                            let png = feMask.pngData()
                        else { return }
                        tiles[isFront ? 0 : 1] = png
                    }
                }
            } else {
                for faceIdx in 0..<8 {
                    autoreleasepool {
                        if let tile = slicer.sliceTile(
                            at: faceIdx, from: mask
                        ),
                            let png = tile.pngData()
                        {
                            tiles[faceIdx] = png
                        }
                    }
                }
            }
            precomputedMaskTiles[camId] = tiles
        }

        if useFisheye {
            let frontDir = imagesDir.appendingPathComponent("fisheye_front")
            let backDir = imagesDir.appendingPathComponent("fisheye_back")
            let frontMaskDir = masksDir.appendingPathComponent("fisheye_front")
            let backMaskDir = masksDir.appendingPathComponent("fisheye_back")
            try fm.createDirectory(at: frontDir, withIntermediateDirectories: true)
            try fm.createDirectory(at: backDir, withIntermediateDirectories: true)
            try fm.createDirectory(at: frontMaskDir, withIntermediateDirectories: true)
            try fm.createDirectory(at: backMaskDir, withIntermediateDirectories: true)
        }

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
                let jpegSize = Self.imageDimensions(dst)
                Self.writeWhiteMask(
                    to: maskDst,
                    width: jpegSize?.width ?? scan.imageHeight,
                    height: jpegSize?.height ?? scan.imageWidth
                )
            }

            // Smoothed depth
            if let src = scan.smoothedDepthBinURL, fm.fileExists(atPath: src.path) {
                let dst = depthsDir.appendingPathComponent(
                    "\(scan.scanName)_depth_smoothed.bin"
                )
                try? fm.copyItem(at: src, to: dst)
            }

            // Insta360 images
            for (camId, erpURL) in scan.insta360ImageURLs {
                // Stitch inside inner autoreleasepool so raw
                // dual-fisheye data (~89 MB) is freed immediately.
                let stitchedERP: UIImage? = autoreleasepool {
                    guard let erpData = try? Data(
                        contentsOf: erpURL
                    ),
                        let rawImage = UIImage(data: erpData)
                    else { return nil as UIImage? }

                    if DualFisheyeStitcher.isDualFisheye(
                        rawImage
                    ) {
                        let model = cameraConfig.cameras
                            .first { $0.id == camId }?
                            .model ?? ""
                        let roll = DualFisheyeStitcher
                            .rollCorrection(forModel: model)
                        return DualFisheyeStitcher.stitch(
                            rawImage, outputWidth: 2048,
                            rollDeg: roll
                        )
                    } else {
                        return rawImage
                    }
                }
                guard let stitchedERP else { continue }

                let erpImage: UIImage = autoreleasepool {
                    if let cp = colorProfile {
                        return ColorProfileCalibrator.apply(
                            cp, to: stitchedERP
                        )
                    }
                    return stitchedERP
                }

                let erpMask = invertedMasks[camId]

                let camTiles = precomputedMaskTiles[camId]

                if useFisheye {
                    let feSize = Self.fisheyeSize
                    for isFront in [true, false] {
                        autoreleasepool {
                            let subdir = isFront
                                ? "fisheye_front"
                                : "fisheye_back"
                            guard let (feImage, feMask) =
                                Self.reprojectERPtoFisheye(
                                    erpImage: erpImage,
                                    erpMask: erpMask,
                                    fisheyeSize: feSize,
                                    isFrontLens: isFront
                                ) else { return }

                            let imgDst = imagesDir
                                .appendingPathComponent(subdir)
                                .appendingPathComponent(
                                    "\(scan.scanName).jpg"
                                )
                            if let jpegData =
                                feImage.jpegData(
                                    compressionQuality: 0.92
                                )
                            {
                                try? jpegData.write(
                                    to: imgDst
                                )
                            }

                            let mskDst = masksDir
                                .appendingPathComponent(subdir)
                                .appendingPathComponent(
                                    "\(scan.scanName).png"
                                )
                            if let png = camTiles?[
                                isFront ? 0 : 1
                            ] {
                                try? png.write(to: mskDst)
                            } else if let png =
                                feMask.pngData()
                            {
                                try? png.write(to: mskDst)
                            }
                        }
                    }
                } else {
                    for faceIdx in 0..<8 {
                        autoreleasepool {
                            let faceDir = imagesDir
                                .appendingPathComponent(
                                    String(
                                        format: "face_%02d",
                                        faceIdx
                                    )
                                )
                            try? fm.createDirectory(
                                at: faceDir,
                                withIntermediateDirectories:
                                    true
                            )
                            let dst = faceDir
                                .appendingPathComponent(
                                    "\(scan.scanName).jpg"
                                )
                            if let tile = slicer.sliceTile(
                                at: faceIdx,
                                from: erpImage
                            ),
                                let jpegData =
                                    tile.jpegData(
                                        compressionQuality:
                                            0.92
                                    )
                            {
                                try? jpegData.write(to: dst)
                            }

                            let maskFaceDir = masksDir
                                .appendingPathComponent(
                                    String(
                                        format: "face_%02d",
                                        faceIdx
                                    )
                                )
                            try? fm.createDirectory(
                                at: maskFaceDir,
                                withIntermediateDirectories:
                                    true
                            )
                            let mskDst = maskFaceDir
                                .appendingPathComponent(
                                    "\(scan.scanName).png"
                                )
                            if let png =
                                camTiles?[faceIdx]
                            {
                                try? png.write(to: mskDst)
                            } else {
                                Self.writeWhiteMask(
                                    to: mskDst,
                                    width: slicer.tileSize,
                                    height: slicer.tileSize
                                )
                            }
                        }
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

    private static func imageDimensions(
        _ url: URL
    ) -> (width: Int, height: Int)? {
        guard let source = CGImageSourceCreateWithURL(
            url as CFURL, nil
        ) else { return nil }
        guard let props = CGImageSourceCopyPropertiesAtIndex(
            source, 0, nil
        ) as? [CFString: Any] else { return nil }
        guard let w = props[kCGImagePropertyPixelWidth] as? Int,
              let h = props[kCGImagePropertyPixelHeight] as? Int
        else { return nil }
        return (w, h)
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

    // MARK: - ERP → Fisheye reprojection

    /// Reprojects an equirectangular image to an equidistant fisheye image.
    /// Returns the fisheye image and its circular mask.
    private static func reprojectERPtoFisheye(
        erpImage: UIImage,
        erpMask: UIImage?,
        fisheyeSize: Int,
        isFrontLens: Bool
    ) -> (image: UIImage, mask: UIImage)? {
        guard let erpCG = erpImage.cgImage else { return nil }
        let erpW = erpCG.width, erpH = erpCG.height
        guard erpW > 0, erpH > 0 else { return nil }

        // Read ERP pixels
        var erpPx = [UInt8](repeating: 0, count: erpW * erpH * 4)
        guard let erpCtx = CGContext(
            data: &erpPx, width: erpW, height: erpH,
            bitsPerComponent: 8, bytesPerRow: erpW * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue
        ) else { return nil }
        erpCtx.draw(erpCG, in: CGRect(x: 0, y: 0, width: erpW, height: erpH))

        // Read ERP mask pixels (if available)
        var maskPx: [UInt8]?
        if let maskCG = erpMask?.cgImage {
            var mp = [UInt8](repeating: 255, count: erpW * erpH * 4)
            if let mCtx = CGContext(
                data: &mp, width: erpW, height: erpH,
                bitsPerComponent: 8, bytesPerRow: erpW * 4,
                space: CGColorSpaceCreateDeviceRGB(),
                bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue
            ) {
                mCtx.draw(maskCG, in: CGRect(x: 0, y: 0, width: erpW, height: erpH))
                maskPx = mp
            }
        }

        // Fisheye model parameters
        let radius = Float(fisheyeSize) * fisheyeRadiusFrac
        let focal = radius / fisheyeHalfFovRad
        let cx = Float(fisheyeSize) / 2.0
        let cy = Float(fisheyeSize) / 2.0

        let S = fisheyeSize
        var outPx = [UInt8](repeating: 0, count: S * S * 4)
        var outMask = [UInt8](repeating: 0, count: S * S * 4)

        let erpWf = Float(erpW), erpHf = Float(erpH)
        let erpStride = erpW * 4
        let zSign: Float = isFrontLens ? 1.0 : -1.0
        let xSign: Float = isFrontLens ? 1.0 : -1.0

        for v in 0..<S {
            for u in 0..<S {
                let dx = Float(u) - cx
                let dy = Float(v) - cy
                let r = sqrtf(dx * dx + dy * dy)

                let idx = (v * S + u) * 4

                guard r < radius, r > 0.001 else {
                    if r <= 0.001 {
                        // Center pixel — sample ERP center of hemisphere
                        let eu: Float = isFrontLens ? erpWf * 0.5 : 0
                        let ev: Float = erpHf * 0.5
                        let eix = min(Int(eu), erpW - 1)
                        let eiy = min(Int(ev), erpH - 1)
                        let eOff = eiy * erpStride + eix * 4
                        outPx[idx] = erpPx[eOff]
                        outPx[idx + 1] = erpPx[eOff + 1]
                        outPx[idx + 2] = erpPx[eOff + 2]
                        outPx[idx + 3] = 255
                        outMask[idx] = 255
                        outMask[idx + 1] = 255
                        outMask[idx + 2] = 255
                        outMask[idx + 3] = 255
                    }
                    continue
                }

                let theta = r / focal
                let sinT = sinf(theta)
                let cosT = cosf(theta)

                // Camera ray → world ray (ERP frame)
                let bx = xSign * sinT * dx / r
                let by = sinT * dy / r
                let bz = zSign * cosT

                // World ray → ERP (lat, lon)
                let lat = asinf(max(-1, min(1, -by)))
                let lon = atan2f(bx, bz)

                // ERP pixel coordinates
                let eu = (lon / (2 * .pi) + 0.5) * erpWf
                let ev = (0.5 - lat / .pi) * erpHf

                // Bilinear sample
                let eix = min(max(Int(eu), 0), erpW - 2)
                let eiy = min(max(Int(ev), 0), erpH - 2)
                let fx = eu - Float(eix)
                let fy = ev - Float(eiy)
                let w00 = (1 - fx) * (1 - fy)
                let w10 = fx * (1 - fy)
                let w01 = (1 - fx) * fy
                let w11 = fx * fy
                let base = eiy * erpStride + eix * 4

                var rv: Float = 0, gv: Float = 0, bv: Float = 0
                for (off, wt) in [(base, w00),
                                   (base + 4, w10),
                                   (base + erpStride, w01),
                                   (base + erpStride + 4, w11)] {
                    rv += Float(erpPx[off]) * wt
                    gv += Float(erpPx[off + 1]) * wt
                    bv += Float(erpPx[off + 2]) * wt
                }
                outPx[idx] = UInt8(clamping: Int(rv))
                outPx[idx + 1] = UInt8(clamping: Int(gv))
                outPx[idx + 2] = UInt8(clamping: Int(bv))
                outPx[idx + 3] = 255

                // Mask: white if inside circle AND ERP mask is valid
                var maskVal: UInt8 = 255
                if let mp = maskPx {
                    let mOff = eiy * erpStride + eix * 4
                    maskVal = mp[mOff]
                }
                outMask[idx] = maskVal
                outMask[idx + 1] = maskVal
                outMask[idx + 2] = maskVal
                outMask[idx + 3] = 255
            }
        }

        let outStride = S * 4
        guard let imgCtx = CGContext(
            data: &outPx, width: S, height: S,
            bitsPerComponent: 8, bytesPerRow: outStride,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue
        ), let imgCG = imgCtx.makeImage() else { return nil }

        guard let maskCtx = CGContext(
            data: &outMask, width: S, height: S,
            bitsPerComponent: 8, bytesPerRow: outStride,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue
        ), let maskCG = maskCtx.makeImage() else { return nil }

        return (UIImage(cgImage: imgCG), UIImage(cgImage: maskCG))
    }

    // MARK: - Blur detection

    static func blurScore(
        imageURL: URL
    ) -> Float? {
        guard
            let imgData = try? Data(
                contentsOf: imageURL
            ),
            let uiImage = UIImage(data: imgData),
            let cgImage = uiImage.cgImage
        else { return nil }

        let srcW = cgImage.width
        let srcH = cgImage.height
        guard srcW > 0, srcH > 0 else { return nil }

        let grayCount = srcW * srcH
        let grayPtr = UnsafeMutablePointer<UInt8>
            .allocate(capacity: grayCount)
        defer { grayPtr.deallocate() }

        let graySpace =
            CGColorSpaceCreateDeviceGray()
        guard let ctx = CGContext(
            data: grayPtr,
            width: srcW, height: srcH,
            bitsPerComponent: 8,
            bytesPerRow: srcW,
            space: graySpace,
            bitmapInfo: CGImageAlphaInfo
                .none.rawValue
        ) else { return nil }
        ctx.draw(
            cgImage,
            in: CGRect(
                x: 0, y: 0,
                width: srcW, height: srcH
            )
        )

        let targetW = 320
        let targetH = Int(
            Float(targetW) * Float(srcH)
                / Float(srcW)
        )
        guard targetW > 2, targetH > 2
        else { return nil }

        let resCount = targetW * targetH
        let resPtr = UnsafeMutablePointer<UInt8>
            .allocate(capacity: resCount)
        defer { resPtr.deallocate() }

        var srcBuf = vImage_Buffer(
            data: grayPtr,
            height: vImagePixelCount(srcH),
            width: vImagePixelCount(srcW),
            rowBytes: srcW
        )
        var resBuf = vImage_Buffer(
            data: resPtr,
            height: vImagePixelCount(targetH),
            width: vImagePixelCount(targetW),
            rowBytes: targetW
        )
        guard vImageScale_Planar8(
            &srcBuf, &resBuf, nil,
            vImage_Flags(kvImageNoFlags)
        ) == kvImageNoError else { return nil }

        let floatRow =
            targetW * MemoryLayout<Float>.size
        let floatPtr = UnsafeMutablePointer<Float>
            .allocate(capacity: resCount)
        defer { floatPtr.deallocate() }

        var floatBuf = vImage_Buffer(
            data: floatPtr,
            height: vImagePixelCount(targetH),
            width: vImagePixelCount(targetW),
            rowBytes: floatRow
        )
        vImageConvert_Planar8toPlanarF(
            &resBuf, &floatBuf,
            255.0, 0.0,
            vImage_Flags(kvImageNoFlags)
        )

        var kernel: [Float] = [
            0, 1, 0,
            1, -4, 1,
            0, 1, 0,
        ]
        let lapPtr = UnsafeMutablePointer<Float>
            .allocate(capacity: resCount)
        defer { lapPtr.deallocate() }

        var outBuf = vImage_Buffer(
            data: lapPtr,
            height: vImagePixelCount(targetH),
            width: vImagePixelCount(targetW),
            rowBytes: floatRow
        )
        guard vImageConvolve_PlanarF(
            &floatBuf, &outBuf, nil,
            0, 0,
            &kernel, 3, 3,
            0,
            vImage_Flags(kvImageEdgeExtend)
        ) == kvImageNoError else { return nil }

        var mean: Float = 0
        var meanSq: Float = 0
        let len = vDSP_Length(resCount)
        vDSP_meanv(lapPtr, 1, &mean, len)
        vDSP_measqv(lapPtr, 1, &meanSq, len)
        return meanSq - mean * mean
    }

    // MARK: - On-device triangulation

    /// Runs SIFT feature extraction, sequential matching, and
    /// triangulation on iPhone images using known ARKit poses.
    /// Rewrites images.bin and points3D.bin with tracked points.
    func triangulate(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig,
        cameraExportMode: CameraExportMode = .pinholeTiles
    ) throws {
        guard let first = scans.first, scans.count >= 2 else {
            print("[Triangulate] need ≥2 scans")
            return
        }

        let insta360ImagesPerScan: Int
        switch cameraExportMode {
        case .nativeFisheye: insta360ImagesPerScan = cameraConfig.cameras.count * 2
        case .pinholeTiles:  insta360ImagesPerScan = cameraConfig.cameras.count * 8
        }

        // Portrait intrinsics (matches cameras.bin)
        let imgW_l = first.imageWidth
        let imgH_l = first.imageHeight
        let fx_l = first.intrinsics[0][0]
        let fy_l = first.intrinsics[1][1]
        let cx_l = first.intrinsics[0][2]
        let cy_l = first.intrinsics[1][2]

        let fx_p = fy_l
        let fy_p = fx_l
        let cx_p = Float(imgH_l - 1) - cy_l
        let cy_p = cx_l

        let K = simd_float3x3(rows: [
            SIMD3(fx_p, 0, cx_p),
            SIMD3(0, fy_p, cy_p),
            SIMD3(0, 0, 1),
        ])

        let iphoneDir = imagesDir
            .appendingPathComponent("face_iphone")

        var imagePaths: [String] = []
        var projMatrices: [Data] = []
        var imageIdMap: [Int: UInt32] = [:]
        var imageId: UInt32 = 1

        for (idx, scan) in scans.enumerated() {
            let path = iphoneDir
                .appendingPathComponent(
                    "\(scan.scanName).jpg"
                ).path
            guard FileManager.default
                .fileExists(atPath: path)
            else {
                imageId += 1
                if !scan.insta360ImageURLs.isEmpty {
                    imageId += UInt32(insta360ImagesPerScan)
                }
                continue
            }

            let T_wc = scan.arkitPose
            let (R_w2c_l, t_w2c_mat) =
                Self.arkitToColmapW2C(T_wc)
            let t_w2c_l = SIMD3(
                t_w2c_mat.columns.3.x,
                t_w2c_mat.columns.3.y,
                t_w2c_mat.columns.3.z
            )
            let R_w2c = Self.portraitRotation * R_w2c_l
            let t_w2c = Self.portraitRotation * t_w2c_l

            // P = K * [R | t]  (3×4, row-major floats)
            let KR = K * R_w2c
            let Kt = K * t_w2c
            var floats: [Float] = [
                KR.columns.0.x, KR.columns.1.x,
                KR.columns.2.x, Kt.x,
                KR.columns.0.y, KR.columns.1.y,
                KR.columns.2.y, Kt.y,
                KR.columns.0.z, KR.columns.1.z,
                KR.columns.2.z, Kt.z,
            ]
            let projData = Data(
                bytes: &floats,
                count: 12 * MemoryLayout<Float>.size
            )

            imageIdMap[imagePaths.count] = imageId
            imagePaths.append(path)
            projMatrices.append(projData)

            imageId += 1
            if !scan.insta360ImageURLs.isEmpty {
                let tileCamCount = cameraConfig
                    .cameras.count * 8
                imageId += UInt32(tileCamCount)
            }
        }

        guard imagePaths.count >= 2 else {
            print("[Triangulate] <2 valid images")
            return
        }

        print("[Triangulate] running SIFT on "
              + "\(imagePaths.count) images…")

        let triangulator = SfMTriangulator()
        var sfmError: NSError?
        let nPts = triangulator.safeTriangulateImages(
            imagePaths, projectionMatrices: projMatrices,
            error: &sfmError
        )
        if let sfmError {
            print("[Triangulate] failed: \(sfmError)")
            return
        }
        guard nPts > 0 else {
            print("[Triangulate] no points triangulated")
            return
        }
        print("[Triangulate] \(nPts) points, writing…")

        // Build tracked points and observation map
        var trackedPoints: [TrackedPoint] = []
        // imageId → [(px, py, point3DId)]
        var obsMap: [UInt32: [(Float, Float, UInt64)]] = [:]

        for i in 0..<nPts {
            var x: Float = 0, y: Float = 0, z: Float = 0
            var r: UInt8 = 0, g: UInt8 = 0, b: UInt8 = 0
            var err: Float = 0
            triangulator.point(
                i, x: &x, y: &y, z: &z,
                r: &r, g: &g, b: &b, error: &err
            )

            let ptId = UInt64(i + 1)
            let trackLen = triangulator
                .trackLength(forPoint: i)
            var track: [(UInt32, Float, Float)] = []

            for j in 0..<trackLen {
                var imgIdx: Int32 = 0
                var px: Float = 0, py: Float = 0
                triangulator.observation(
                    j, forPoint: i,
                    imageIndex: &imgIdx,
                    px: &px, py: &py
                )
                guard let imId = imageIdMap[
                    Int(imgIdx)
                ] else { continue }
                track.append((imId, px, py))
                obsMap[imId, default: []]
                    .append((px, py, ptId))
            }

            trackedPoints.append(TrackedPoint(
                id: ptId,
                position: SIMD3(x, y, z),
                r: r, g: g, b: b,
                error: err, track: track
            ))
        }

        // Rewrite points3D.bin with tracks
        try writeTrackedPoints3DBin(
            points: trackedPoints
        )

        // Rewrite images.bin with observations
        try rewriteImagesBinWithObs(
            scans: scans,
            cameraConfig: cameraConfig,
            obsMap: obsMap,
            cameraExportMode: cameraExportMode
        )

        print("[Triangulate] done — \(nPts) points "
              + "with tracks written")
    }

    private func writeTrackedPoints3DBin(
        points: [TrackedPoint]
    ) throws {
        let url = sparseDir.appendingPathComponent(
            "points3D.bin"
        )
        FileManager.default.createFile(
            atPath: url.path, contents: nil
        )
        let handle = try FileHandle(forWritingTo: url)
        defer { try? handle.close() }

        var header = Data()
        header.appendLE(UInt64(points.count))
        handle.write(header)

        for p in points {
            var chunk = Data()
            chunk.appendLE(p.id)
            chunk.appendLE(Double(p.position.x))
            chunk.appendLE(Double(p.position.y))
            chunk.appendLE(Double(p.position.z))
            chunk.append(contentsOf: [p.r, p.g, p.b])
            chunk.appendLE(Double(p.error))
            chunk.appendLE(UInt64(p.track.count))
            for (imId, px, py) in p.track {
                chunk.appendLE(imId)
                chunk.appendLE(UInt64(0)) // point2D_idx
            }
            handle.write(chunk)
        }
    }

    private func rewriteImagesBinWithObs(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig,
        obsMap: [UInt32: [(Float, Float, UInt64)]],
        cameraExportMode: CameraExportMode
    ) throws {
        let faceRotations = Self.computeFaceRotations()
        let iphoneCameraId: UInt32 = 1
        let faceCount = 8
        let hasInsta360 = scans.contains {
            !$0.insta360ImageURLs.isEmpty
        }
        let useFisheye = cameraExportMode == .nativeFisheye && hasInsta360

        let tileCameraIds: [Int: UInt32]
        let fisheyeCameraIds: [(front: UInt32, back: UInt32)]
        if useFisheye {
            tileCameraIds = [:]
            fisheyeCameraIds = cameraConfig.cameras.enumerated().map { (i, _) in
                (front: UInt32(2 + i * 2), back: UInt32(3 + i * 2))
            }
        } else {
            tileCameraIds = hasInsta360
                ? Dictionary(uniqueKeysWithValues:
                    (0..<faceCount).map { ($0, UInt32($0 + 2)) })
                : [:]
            fisheyeCameraIds = []
        }

        var entries: [ImageEntry] = []
        var imageId: UInt32 = 1

        for scan in scans {
            let T_wc = scan.arkitPose
            let (R_w2c_l, t_w2c_mat) =
                Self.arkitToColmapW2C(T_wc)
            let t_w2c_l = SIMD3(
                t_w2c_mat.columns.3.x,
                t_w2c_mat.columns.3.y,
                t_w2c_mat.columns.3.z
            )

            let R_w2c_p = Self.portraitRotation * R_w2c_l
            let t_w2c_p = Self.portraitRotation * t_w2c_l

            let obs = obsMap[imageId]?.map {
                (px: $0.0, py: $0.1, point3DId: Int64($0.2))
            } ?? []

            entries.append(ImageEntry(
                imageId: imageId,
                quatWXYZ: Self.rotationToQuatWXYZ(R_w2c_p),
                tvec: t_w2c_p,
                cameraId: iphoneCameraId,
                name: "face_iphone/\(scan.scanName).jpg",
                observations: obs
            ))
            imageId += 1

            if !scan.insta360ImageURLs.isEmpty {
                for (camIdx, camCfg) in cameraConfig.cameras.enumerated() {
                    let T_insta_arkit = Self.erpFromARKit(
                        camCfg.extrinsic
                    )
                    let T_insta_world =
                        T_insta_arkit * T_wc.inverse
                    let R_iw = simd_float3x3(
                        SIMD3(T_insta_world.columns.0.x,
                              T_insta_world.columns.0.y,
                              T_insta_world.columns.0.z),
                        SIMD3(T_insta_world.columns.1.x,
                              T_insta_world.columns.1.y,
                              T_insta_world.columns.1.z),
                        SIMD3(T_insta_world.columns.2.x,
                              T_insta_world.columns.2.y,
                              T_insta_world.columns.2.z)
                    )
                    let t_i = SIMD3(
                        T_insta_world.columns.3.x,
                        T_insta_world.columns.3.y,
                        T_insta_world.columns.3.z
                    )
                    let C_i = -R_iw.transpose * t_i
                    let C_col = Self.arkitToColmap * C_i
                    let R_col = R_iw * Self.arkitToColmap

                    if useFisheye {
                        let ids = fisheyeCameraIds[camIdx]

                        let R_front = R_col
                        let t_front = -(R_front * C_col)
                        entries.append(ImageEntry(
                            imageId: imageId,
                            quatWXYZ: Self.rotationToQuatWXYZ(R_front),
                            tvec: t_front,
                            cameraId: ids.front,
                            name: "fisheye_front/\(scan.scanName).jpg",
                            observations: []
                        ))
                        imageId += 1

                        let R_back = Self.R_y_pi * R_col
                        let t_back = -(R_back * C_col)
                        entries.append(ImageEntry(
                            imageId: imageId,
                            quatWXYZ: Self.rotationToQuatWXYZ(R_back),
                            tvec: t_back,
                            cameraId: ids.back,
                            name: "fisheye_back/\(scan.scanName).jpg",
                            observations: []
                        ))
                        imageId += 1
                    } else {
                        for (fIdx, cfp) in
                            faceRotations.enumerated()
                        {
                            guard let cId = tileCameraIds[fIdx]
                            else { continue }
                            let R_t = cfp * R_col
                            let t_t = -(R_t * C_col)
                            entries.append(ImageEntry(
                                imageId: imageId,
                                quatWXYZ: Self.rotationToQuatWXYZ(R_t),
                                tvec: t_t,
                                cameraId: cId,
                                name: "face_\(String(format: "%02d", fIdx))/\(scan.scanName).jpg",
                                observations: []
                            ))
                            imageId += 1
                        }
                    }
                }
            }
        }

        try writeImagesBinWithObs(entries: entries)
    }

    private func writeImagesBinWithObs(
        entries: [ImageEntry]
    ) throws {
        var data = Data()
        data.appendLE(UInt64(entries.count))
        for e in entries {
            data.appendLE(e.imageId)
            for v in e.quatWXYZ {
                data.appendLE(Double(v))
            }
            data.appendLE(Double(e.tvec.x))
            data.appendLE(Double(e.tvec.y))
            data.appendLE(Double(e.tvec.z))
            data.appendLE(e.cameraId)
            data.append(e.name.data(using: .utf8)!)
            data.append(0)
            data.appendLE(
                UInt64(e.observations.count)
            )
            for obs in e.observations {
                data.appendLE(Double(obs.px))
                data.appendLE(Double(obs.py))
                data.appendLE(
                    Int64(obs.point3DId)
                )
            }
        }
        try data.write(
            to: sparseDir
                .appendingPathComponent("images.bin")
        )
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
        let R_c2w_col = arkitToColmap * R_c2w_arkit * arkitToColmap
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
    var observations: [(px: Float, py: Float, point3DId: Int64)]
}

private struct UnprojectedPoint {
    let worldPos: SIMD3<Float>
    let portraitX: Float
    let portraitY: Float
}

private struct ColoredPoint {
    let position: SIMD3<Float>
    let r: UInt8
    let g: UInt8
    let b: UInt8
}

private struct TrackedPoint {
    let id: UInt64
    let position: SIMD3<Float>
    let r: UInt8
    let g: UInt8
    let b: UInt8
    let error: Float
    let track: [(UInt32, Float, Float)]
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
