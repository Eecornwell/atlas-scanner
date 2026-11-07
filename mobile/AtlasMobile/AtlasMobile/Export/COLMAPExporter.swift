import Accelerate
import Foundation
import ImageIO
import os
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
    private static let fisheyeSize = 2048
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

    // COLMAP camera frame (X=right,Y=down,Z=forward) from
    // ARKit camera frame (X=right,Y=up,Z=backward).
    private static let cameraFromARKit = simd_float3x3(rows: [
        SIMD3<Float>(1,  0,  0),
        SIMD3<Float>(0, -1,  0),
        SIMD3<Float>(0,  0, -1),
    ])

    /// Rig rotation correction from on-device RigCalibrator.
    /// Set before export via `setRigCorrection(_:)`.
    private static var rigCorrection = matrix_identity_float3x3

    static func setRigCorrection(
        _ correction: simd_float3x3
    ) {
        rigCorrection = correction
    }

    static func resetRigCorrection() {
        rigCorrection = matrix_identity_float3x3
    }

    static let tileSize = 2048
    static let tileFovDeg: Float = 65.0
    static let blurThreshold: Float = 300.0
    /// X5 shots scoring below min(x5BlurThreshold, x5BlurMedianFraction
    /// x session median) are dropped. 1000 dropped 9 of 13 sharp shots
    /// of an open field (scores 560-820).
    static let x5BlurThreshold: Float = 350.0
    static let x5BlurMedianFraction: Float = 0.5
    /// X5 shots whose recorded mean angular rate (over the 1.5 s after
    /// the shutter command) exceeds this are dropped: their pose,
    /// interpolated at command + x5ShutterDelay, is unreliable. Shots
    /// in a 9-minute walk were mostly 0.6-2 deg/s; bad ones 3.2-7.7.
    static let x5MaxMeanDps: Double = 3.0
    static let x5ShutterDelay: Double = 1.0

    /// Seed extrinsic → ERP-from-ARKit 4x4 for composing with T_wc.inverse.
    static func erpFromARKit(_ ext: RigidTransform) -> simd_float4x4 {
        let T = ExtrinsicOptimizer.seedToCameraMatrix(ext)
        let R = simd_float3x3(
            SIMD3(T.columns.0.x, T.columns.0.y, T.columns.0.z),
            SIMD3(T.columns.1.x, T.columns.1.y, T.columns.1.z),
            SIMD3(T.columns.2.x, T.columns.2.y, T.columns.2.z)
        )
        let Rc = R * rigCorrection * cameraFromARKit
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

    /// The export holds one Insta360 camera: image names such as
    /// face_00/scan_001.jpg, the tile camera ids and the rig carry no
    /// camera id. Keeps the camera entry with the most 360 images in the
    /// session and leaves out other cameras' images. With two entries in
    /// Camera Settings the export wrote every 360 image name twice (a
    /// crash in writeFramesBin), and rig calibration used the first
    /// entry's mount position whichever camera took the photos.
    static func singleInsta360Camera(
        scans: [ScanExportData],
        config: MultiCameraConfig
    ) -> (scans: [ScanExportData],
          config: MultiCameraConfig,
          note: String) {
        var counts: [String: Int] = [:]
        for scan in scans {
            for id in scan.insta360ImageURLs.keys {
                counts[id, default: 0] += 1
            }
        }
        // Most images first; ties go to the earlier entry.
        let order = config.cameras.map(\.id)
        let keepId = counts.keys.min { a, b in
            let na = counts[a] ?? 0
            let nb = counts[b] ?? 0
            if na != nb { return na > nb }
            let ia = order.firstIndex(of: a) ?? Int.max
            let ib = order.firstIndex(of: b) ?? Int.max
            return ia != ib ? ia < ib : a < b
        }
        guard var entry = config.cameras.first(where: {
            $0.id == keepId
        }) ?? config.cameras.first(where: \.enabled)
            ?? config.cameras.first
        else {
            return (scans, config, "no camera entries")
        }
        // The shots exist, so export and calibrate them even if the
        // entry was switched off after the session.
        entry.enabled = true

        var leftOut = 0
        let kept = scans.map { scan -> ScanExportData in
            let urls = scan.insta360ImageURLs.filter {
                $0.key == keepId
            }
            guard urls.count != scan.insta360ImageURLs.count
            else { return scan }
            leftOut += scan.insta360ImageURLs.count - urls.count
            return ScanExportData(
                scanName: scan.scanName,
                arkitPose: scan.arkitPose,
                intrinsics: scan.intrinsics,
                imageWidth: scan.imageWidth,
                imageHeight: scan.imageHeight,
                depthBinURL: scan.depthBinURL,
                smoothedDepthBinURL: scan.smoothedDepthBinURL,
                iphoneImageURL: scan.iphoneImageURL,
                insta360ImageURLs: urls,
                insta360CommandTimestamp:
                    scan.insta360CommandTimestamp)
        }
        var note = "360 camera: \(entry.id) (\(entry.model))"
        if let keepId {
            note += ", \(counts[keepId] ?? 0) images"
            if keepId != entry.id { note += " in raw/\(keepId)" }
        }
        if leftOut > 0 {
            note += "; left out \(leftOut) images of other cameras"
        }
        return (kept,
                MultiCameraConfig(cameras: [entry],
                                  iphone: config.iphone),
                note)
    }

    /// Writes cameras.bin, images.bin, points3D.bin, rigs.bin, and frames.bin.
    /// `onStage` receives a short description of each step, called on
    /// the exporting thread.
    func export(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig,
        cameraExportMode: CameraExportMode = .pinholeTiles,
        onStage: ((String) -> Void)? = nil
    ) throws {
        let rcQ = simd_quatf(Self.rigCorrection)
        let rcDeg = 2 * acos(min(abs(rcQ.real), 1.0))
            * 180.0 / Float.pi
        let s = 2.0 * 180.0 / Float.pi
        print(String(
            format: "[Export] rigCorrection: %.2f°"
                + " rv=[%.3f, %.3f, %.3f]°"
                + " q=[%.6f, %.6f, %.6f, %.6f]",
            rcDeg,
            rcQ.imag.x * s, rcQ.imag.y * s,
            rcQ.imag.z * s,
            rcQ.real, rcQ.imag.x,
            rcQ.imag.y, rcQ.imag.z
        ))
        try FileManager.default.createDirectory(
            at: sparseDir, withIntermediateDirectories: true)

        guard let first = scans.first else { return }

        let tileSize = Self.tileSize
        let tileFovDeg = Self.tileFovDeg
        let f_tile = Float(tileSize) / (2.0 * tan(tileFovDeg * .pi / 360.0))
        let c_tile = Float(tileSize) / 2.0

        let iphoneCameraId: UInt32 = 1
        let faceCount = 8

        onStage?("checking sharpness (\(Self.memorySummary()))")

        // Score every image first, so the X5 limit can follow the
        // session's median.
        var iphoneScores: [Float?] = []
        var x5Scores: [[String: Float]] = []
        var x5Rates: [[String: Double]] = []
        for scan in scans {
            autoreleasepool {
                iphoneScores.append(scan.iphoneImageURL.flatMap {
                    Self.blurScore(imageURL: $0)
                })
                var shots: [String: Float] = [:]
                for (camId, url) in scan.insta360ImageURLs {
                    shots[camId] = Self.blurScore(imageURL: url)
                }
                x5Scores.append(shots)
                x5Rates.append(
                    scan.insta360ImageURLs.isEmpty
                        ? [:]
                        : x5MeanRates(scanName: scan.scanName))
            }
        }
        // The score (Laplacian variance) measures fine texture, so it
        // is low for sharp X5 shots of sky and grass as well as for
        // blurred ones. Use a low fixed limit, lowered further for
        // sessions whose shots all score low.
        let allX5 = x5Scores.flatMap { $0.values }.sorted()
        let x5Median = allX5.isEmpty ? 0 : allX5[allX5.count / 2]
        let x5Limit = min(Self.x5BlurThreshold,
                          Self.x5BlurMedianFraction * x5Median)
        print(String(
            format: "[Export] X5 blur limit %.0f"
                + " (median %.0f over %ld shots)",
            x5Limit, x5Median, allX5.count))

        // A blurry iPhone frame drops the whole scan: the iPhone is
        // the rig reference. A low-scoring X5 shot drops only that
        // shot and keeps the scan's iPhone frame.
        var filteredScans: [ScanExportData] = []
        var skippedNames: [String] = []
        var droppedX5: [String] = []
        for (i, scan) in scans.enumerated() {
            if let score = iphoneScores[i],
               score < Self.blurThreshold
            {
                let tag = String(format: "%.1f", score)
                skippedNames.append(
                    "\(scan.scanName) iphone(\(tag))")
                continue
            }
            var keptX5 = scan.insta360ImageURLs
            for (camId, score) in x5Scores[i]
            where score < x5Limit {
                keptX5[camId] = nil
                let tag = String(format: "%.1f", score)
                droppedX5.append(
                    "\(scan.scanName) x5-\(camId)(\(tag))")
            }
            for (camId, rate) in x5Rates[i]
            where rate > Self.x5MaxMeanDps && keptX5[camId] != nil {
                keptX5[camId] = nil
                let tag = String(format: "%.1f deg/s", rate)
                droppedX5.append(
                    "\(scan.scanName) x5-\(camId)(\(tag))")
            }
            if keptX5.count == scan.insta360ImageURLs.count {
                filteredScans.append(scan)
            } else {
                filteredScans.append(ScanExportData(
                    scanName: scan.scanName,
                    arkitPose: scan.arkitPose,
                    intrinsics: scan.intrinsics,
                    imageWidth: scan.imageWidth,
                    imageHeight: scan.imageHeight,
                    depthBinURL: scan.depthBinURL,
                    smoothedDepthBinURL:
                        scan.smoothedDepthBinURL,
                    iphoneImageURL: scan.iphoneImageURL,
                    insta360ImageURLs: keptX5,
                    insta360CommandTimestamp:
                        scan.insta360CommandTimestamp))
            }
        }
        if !droppedX5.isEmpty {
            print(
                "[Export] dropped \(droppedX5.count) X5 shots,"
                + " kept their iPhone frames: "
                + droppedX5.joined(separator: ", "))
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

        let trajectory = loadTrajectory()
        if let traj = trajectory {
            print(
                "[Export] trajectory loaded:"
                + " \(traj.count) poses"
            )
        } else {
            print("[Export] no trajectory — using"
                + " scan poses for X5")
        }

        onStage?("writing sparse model (\(Self.memorySummary()))")
        do {
            let (imageEntries, allPoints) =
                buildImageEntriesAndPoints(
                    scans: filteredScans,
                    cameraConfig: cameraConfig,
                    iphoneCameraId: iphoneCameraId,
                    tileCameraIds: tileCameraIds,
                    fisheyeCameraIds: fisheyeCameraIds,
                    trajectory: trajectory,
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
                    fisheyeCameraIds: fisheyeCameraIds,
                    trajectory: trajectory
                )
            }
        }
        try copyImagesAndDepths(
            scans: filteredScans,
            cameraConfig: cameraConfig,
            useFisheye: useFisheye,
            iphoneW: imgW_p,
            iphoneH: imgH_p,
            onStage: onStage
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
        trajectory: [(timestamp: Double, pose: simd_float4x4)]?,
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
                let x5Pose: simd_float4x4
                if let cmdTs = scan.insta360CommandTimestamp,
                   let traj = trajectory,
                   let interpolated = Self.interpolateTrajectory(
                       traj, at: cmdTs + Self.x5ShutterDelay
                   )
                {
                    x5Pose = interpolated
                } else {
                    x5Pose = T_wc
                }

                for (camIdx, camCfg) in cameraConfig.cameras.enumerated() {
                    let T_insta_arkit = Self.erpFromARKit(camCfg.extrinsic)
                    let T_iphone_world = x5Pose.inverse
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
            url: scan.iphoneImageURL,
            maxWidth: 640
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

                let sampleW = pixels?.w ?? imgW
                let sampleH = pixels?.h ?? imgH
                let rgb = Self.sampleColor(
                    pixels: pixels,
                    lidarCol: col, lidarRow: row,
                    imgW: sampleW, imgH: sampleH,
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
        url: URL?,
        maxWidth: Int = 0
    ) -> (data: [UInt8], w: Int, h: Int)? {
        autoreleasepool {
            guard let url,
                  let source =
                      CGImageSourceCreateWithURL(
                          url as CFURL, nil)
            else { return nil }

            let cg: CGImage
            if maxWidth > 0 {
                var opts: [CFString: Any] = [
                    kCGImageSourceThumbnailMaxPixelSize:
                        maxWidth,
                    kCGImageSourceCreateThumbnailFromImageAlways:
                        true,
                    kCGImageSourceShouldCacheImmediately:
                        true,
                    kCGImageSourceShouldCache:
                        false,
                ]
                if maxWidth < 1024 {
                    opts[kCGImageSourceSubsampleFactor]
                        = 4
                } else if let props =
                    CGImageSourceCopyPropertiesAtIndex(
                        source, 0, nil
                    ) as? [CFString: Any],
                    let srcW = props[
                        kCGImagePropertyPixelWidth
                    ] as? Int,
                    srcW > 8000
                {
                    opts[kCGImageSourceSubsampleFactor]
                        = 2
                }
                guard let thumb =
                    CGImageSourceCreateThumbnailAtIndex(
                        source, 0,
                        opts as CFDictionary)
                else { return nil }
                cg = thumb
            } else {
                guard let full =
                    CGImageSourceCreateImageAtIndex(
                        source, 0, [
                            kCGImageSourceShouldCacheImmediately:
                                true,
                            kCGImageSourceShouldCache:
                                false,
                        ] as CFDictionary)
                else { return nil }
                cg = full
            }

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
        // write(contentsOf:) throws on I/O errors such as a full disk.
        // The older write(_:) raises an Objective-C exception instead,
        // which ends the app.
        try handle.write(contentsOf: header)

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
            try handle.write(contentsOf: chunk)
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

            if camIdx == 0 {
                let T_raw = ExtrinsicOptimizer
                    .seedToCameraMatrix(camCfg.extrinsic)
                let R_raw = simd_float3x3(
                    SIMD3(T_raw.columns.0.x, T_raw.columns.0.y, T_raw.columns.0.z),
                    SIMD3(T_raw.columns.1.x, T_raw.columns.1.y, T_raw.columns.1.z),
                    SIMD3(T_raw.columns.2.x, T_raw.columns.2.y, T_raw.columns.2.z)
                )
                let Rc_uncorr = R_raw * Self.cameraFromARKit
                let S_old = Rc_uncorr * Self.arkitToColmap * rRotT
                let S_new = R_insta * Self.arkitToColmap * rRotT
                let delta = S_old.transpose * S_new
                let dq = simd_quatf(delta)
                print(String(
                    format: "[Export] rigDelta:"
                        + " %.2f° axis=[%.3f, %.3f, %.3f]",
                    dq.angle * 180.0 / Float.pi,
                    dq.axis.x, dq.axis.y, dq.axis.z
                ))
            }

            if !fisheyeCameraIds.isEmpty {
                let ids = fisheyeCameraIds[camIdx]

                let Rc = R_insta * Self.arkitToColmap * rRotT
                let t_front = t_insta
                let qF = Self.rotationToQuatWXYZ(Rc)
                data.appendLE(sensorTypeCamera)
                data.appendLE(ids.front)
                data.appendLE(UInt8(1))
                for v in qF { data.appendLE(Double(v)) }
                data.appendLE(Double(t_front.x))
                data.appendLE(Double(t_front.y))
                data.appendLE(Double(t_front.z))

                let R_back = Self.R_y_pi * Rc
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
                    let R_tile = camFromPano * R_insta * Self.arkitToColmap * rRotT
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

    /// Writes frames.bin with separate frames for iPhone and X5.
    /// The iPhone frame uses the scan-time pose; the X5 frame uses
    /// the trajectory-interpolated pose at command + shutter delay,
    /// so tools reading rig files get the corrected X5 position.
    private func writeFramesBin(
        scans: [ScanExportData],
        imageEntries: [ImageEntry],
        cameraConfig: MultiCameraConfig,
        iphoneCameraId: UInt32,
        tileCameraIds: [Int: UInt32],
        fisheyeCameraIds: [(front: UInt32, back: UInt32)],
        trajectory: [(timestamp: Double, pose: simd_float4x4)]?
    ) throws {
        let rigId: UInt32 = 1
        let sensorTypeCamera: Int32 = 0

        let entryByName = Dictionary(
            uniqueKeysWithValues: imageEntries.map {
                ($0.name, $0)
            }
        )

        var data = Data()
        let countOffset = data.count
        data.appendLE(UInt64(0))
        var frameCount: UInt64 = 0

        for scan in scans {
            // iPhone frame at scan time
            let iphoneName =
                "face_iphone/\(scan.scanName).jpg"
            if let e = entryByName[iphoneName] {
                let (R_l, t_mat) = Self.arkitToColmapW2C(
                    scan.arkitPose
                )
                let t_l = SIMD3(
                    t_mat.columns.3.x,
                    t_mat.columns.3.y,
                    t_mat.columns.3.z
                )
                let R = Self.portraitRotation * R_l
                let t = Self.portraitRotation * t_l
                let q = Self.rotationToQuatWXYZ(R)

                frameCount += 1
                data.appendLE(UInt32(frameCount))
                data.appendLE(rigId)
                for v in q { data.appendLE(Double(v)) }
                data.appendLE(Double(t.x))
                data.appendLE(Double(t.y))
                data.appendLE(Double(t.z))
                data.appendLE(UInt32(1))
                data.appendLE(sensorTypeCamera)
                data.appendLE(iphoneCameraId)
                data.appendLE(UInt64(e.imageId))
            }

            // X5 frame at corrected pose
            guard !scan.insta360ImageURLs.isEmpty
            else { continue }

            let x5Pose: simd_float4x4
            if let cmdTs = scan.insta360CommandTimestamp,
               let traj = trajectory,
               let interp = Self.interpolateTrajectory(
                   traj, at: cmdTs + Self.x5ShutterDelay
               )
            {
                x5Pose = interp
            } else {
                x5Pose = scan.arkitPose
            }

            let (R_x5_l, t_x5_mat) =
                Self.arkitToColmapW2C(x5Pose)
            let t_x5_l = SIMD3(
                t_x5_mat.columns.3.x,
                t_x5_mat.columns.3.y,
                t_x5_mat.columns.3.z
            )
            let R_x5 = Self.portraitRotation * R_x5_l
            let t_x5 = Self.portraitRotation * t_x5_l
            let q_x5 = Self.rotationToQuatWXYZ(R_x5)

            var x5Obs: [(Int32, UInt32, UInt64)] = []
            if !fisheyeCameraIds.isEmpty {
                for ids in fisheyeCameraIds {
                    let fn = "fisheye_front/"
                        + "\(scan.scanName).jpg"
                    if let e = entryByName[fn] {
                        x5Obs.append((
                            sensorTypeCamera, ids.front,
                            UInt64(e.imageId)
                        ))
                    }
                    let bn = "fisheye_back/"
                        + "\(scan.scanName).jpg"
                    if let e = entryByName[bn] {
                        x5Obs.append((
                            sensorTypeCamera, ids.back,
                            UInt64(e.imageId)
                        ))
                    }
                }
            } else {
                for _ in cameraConfig.cameras {
                    for (faceIdx, camId) in tileCameraIds
                        .sorted(by: {
                            $0.key < $1.key
                        })
                    {
                        let tn = "face_"
                            + "\(String(format: "%02d", faceIdx))"
                            + "/\(scan.scanName).jpg"
                        if let e = entryByName[tn] {
                            x5Obs.append((
                                sensorTypeCamera, camId,
                                UInt64(e.imageId)
                            ))
                        }
                    }
                }
            }

            guard !x5Obs.isEmpty else { continue }

            frameCount += 1
            data.appendLE(UInt32(frameCount))
            data.appendLE(rigId)
            for v in q_x5 { data.appendLE(Double(v)) }
            data.appendLE(Double(t_x5.x))
            data.appendLE(Double(t_x5.y))
            data.appendLE(Double(t_x5.z))
            data.appendLE(UInt32(x5Obs.count))
            for (sType, camId, imgId) in x5Obs {
                data.appendLE(sType)
                data.appendLE(camId)
                data.appendLE(imgId)
            }
        }

        var countBytes = Data()
        countBytes.appendLE(frameCount)
        data.replaceSubrange(
            countOffset..<countOffset + 8, with: countBytes
        )

        try data.write(
            to: sparseDir.appendingPathComponent(
                "frames.bin"
            )
        )
    }

    // MARK: - Copy images & depths

    private func copyImagesAndDepths(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig,
        useFisheye: Bool,
        iphoneW: Int,
        iphoneH: Int,
        onStage: ((String) -> Void)? = nil
    ) throws {
        let fm = FileManager.default
        let iphoneFaceDir = imagesDir.appendingPathComponent("face_iphone")
        let masksDir = sessionDirectory.appendingPathComponent("colmap/masks")
        try fm.createDirectory(at: iphoneFaceDir, withIntermediateDirectories: true)
        try fm.createDirectory(at: depthsDir, withIntermediateDirectories: true)
        try fm.createDirectory(at: masksDir, withIntermediateDirectories: true)

        let slicer = ERPTileSlicer(
            tileSize: Self.tileSize,
            tileFovDeg: Self.tileFovDeg
        )
        let colorProfile = ColorProfileCalibrator.load()
        if let cp = colorProfile {
            print("[Export] color profile: L×\(cp.lGain) a\(cp.aOffset) b\(cp.bOffset)")
        } else {
            print("[Export] no color profile — exporting uncorrected")
        }

        print("[Export] mem before masks: "
            + "\(Self.memoryMB())MB")
        onStage?("loading masks (\(Self.memorySummary()))")

        // Build precomputed mask tiles. Load each mask
        // one at a time to cap peak memory, and
        // downsample if wider than 4096.
        var precomputedMaskTiles: [String: [Int: Data]] = [:]
        for (camId, maskURL) in loadAllCameraMaskURLs() {
            autoreleasepool {
                let mPx: (px: [UInt8], w: Int, h: Int)?
                    = Self.loadMaskPixels(url: maskURL,
                                          maxWidth: 4096)
                guard let mp = mPx else { return }

                var tiles: [Int: Data] = [:]
                if useFisheye {
                    let feSize = Self.fisheyeSize
                    for isFront in [true, false] {
                        autoreleasepool {
                            // The saved mask is result.maskPx: the
                            // image circle combined with maskPx.
                            // erpPx only feeds result.imagePx,
                            // which is unused here.
                            guard var result =
                                Self.reprojectERPtoFisheye(
                                    erpPx: mp.px,
                                    erpW: mp.w,
                                    erpH: mp.h,
                                    maskPx: mp.px,
                                    fisheyeSize: feSize,
                                    isFrontLens: isFront
                                ),
                                let png =
                                    Self.pngDataFromPixels(
                                        &result.maskPx,
                                        width: feSize,
                                        height: feSize
                                    )
                            else { return }
                            tiles[isFront ? 0 : 1] = png
                        }
                    }
                } else {
                    for faceIdx in 0..<8 {
                        autoreleasepool {
                            if let tile = slicer.sliceTile(
                                at: faceIdx,
                                srcPx: mp.px,
                                srcW: mp.w,
                                srcH: mp.h
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
        }
        slicer.clearCaches()

        print("[Export] mem after masks: "
            + "\(Self.memoryMB())MB")

        let hasInsta360 = scans.contains {
            !$0.insta360ImageURLs.isEmpty
        }
        let iphoneMaskDir = masksDir
            .appendingPathComponent("face_iphone")
        if hasInsta360 {
            try fm.createDirectory(
                at: iphoneMaskDir,
                withIntermediateDirectories: true)
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

        var scanIdx = 0
        for scan in scans { autoreleasepool {
            let mem = Self.memoryMB()
            let mode = useFisheye ? "fisheye" : "tiles"
            print("[Export] scan \(scanIdx)"
                + "/\(scans.count)"
                + " \(scan.scanName)"
                + " mem=\(mem)MB \(mode)")
            onStage?("scan \(scanIdx + 1)/\(scans.count)"
                + " iPhone (\(Self.memorySummary()))")

            // Do not post didReceiveMemoryWarningNotification here.
            // NotificationCenter runs observers on the posting thread,
            // so UIKit, SwiftUI and SDK observers would run on this
            // background export thread. Real memory warnings come
            // from the system on the main thread.
            if scanIdx > 0, scanIdx % 10 == 0 {
                URLCache.shared.removeAllCachedResponses()
                print("[Export] cache flush at scan"
                    + " \(scanIdx) mem=\(mem)MB")
            }
            scanIdx += 1
            // iPhone RGB + mask
            if let src = scan.iphoneImageURL, fm.fileExists(atPath: src.path) {
                let dst = iphoneFaceDir.appendingPathComponent(
                    "\(scan.scanName).jpg"
                )
                try? fm.copyItem(at: src, to: dst)

                if hasInsta360 {
                    Self.writeWhiteMask(
                        to: iphoneMaskDir.appendingPathComponent(
                            "\(scan.scanName).png"),
                        width: iphoneW,
                        height: iphoneH)
                }
            }

            // Smoothed depth
            if let src = scan.smoothedDepthBinURL, fm.fileExists(atPath: src.path) {
                let dst = depthsDir.appendingPathComponent(
                    "\(scan.scanName)_depth_smoothed.bin"
                )
                try? fm.copyItem(at: src, to: dst)
            }

            // Insta360 images
            if !scan.insta360ImageURLs.isEmpty {
                onStage?("scan \(scanIdx)/\(scans.count)"
                    + " 360 image (\(Self.memorySummary()))")
            }
            for (camId, erpURL) in scan.insta360ImageURLs {
                let camTiles = precomputedMaskTiles[camId]

                // Stitch to raw pixels — never creates a
                // UIImage for the ERP, saving ~134 MB peak.
                let buildERPPixels = {
                    [self]
                    () -> (px: [UInt8], w: Int, h: Int)? in
                    guard let source =
                        CGImageSourceCreateWithURL(
                            erpURL as CFURL, nil
                        ),
                        let props =
                            CGImageSourceCopyPropertiesAtIndex(
                                source, 0, nil
                            ) as? [CFString: Any],
                        let imgW = props[
                            kCGImagePropertyPixelWidth
                        ] as? Int,
                        let imgH = props[
                            kCGImagePropertyPixelHeight
                        ] as? Int
                    else { return nil }

                    let ratio = Float(imgW) / Float(imgH)
                    let isDual = ratio > 1.8
                        && ratio < 2.2

                    var pixels: (px: [UInt8],
                                 w: Int, h: Int)?
                    if isDual {
                        let erpW = useFisheye
                            ? min(5120, imgW)
                            : min(8192, imgW)
                        let model = cameraConfig.cameras
                            .first { $0.id == camId }?
                            .model ?? ""
                        let roll = DualFisheyeStitcher
                            .rollCorrection(forModel: model)
                        pixels = DualFisheyeStitcher
                            .stitchToPixels(
                                url: erpURL,
                                outputWidth: erpW,
                                rollDeg: roll,
                                allowSubsample: useFisheye
                            )
                    } else {
                        let img = autoreleasepool {
                            () -> UIImage? in
                            guard let data = try? Data(
                                contentsOf: erpURL
                            ) else { return nil }
                            return UIImage(data: data)
                        }
                        if let img {
                            pixels = Self.extractERPPixels(
                                img
                            )
                        }
                    }
                    guard var px = pixels
                    else { return nil }

                    if isDual {
                        print(
                            "[Export] ERP \(px.w)"
                            + "×\(px.h)"
                            + " from \(imgW)×\(imgH) src"
                        )
                    }

                    if let cp = colorProfile {
                        ColorProfileCalibrator.applyInPlace(
                            cp, pixels: &px.px
                        )
                    }
                    return px
                }

                if useFisheye {
                    let feSize = Self.fisheyeSize
                    let model = cameraConfig.cameras
                        .first { $0.id == camId }?
                        .model ?? ""
                    let roll = DualFisheyeStitcher
                        .rollCorrection(forModel: model)

                    // Check if source is dual-fisheye
                    let isDual: Bool = autoreleasepool {
                        guard let source =
                            CGImageSourceCreateWithURL(
                                erpURL as CFURL, nil),
                            let props =
                                CGImageSourceCopyPropertiesAtIndex(
                                    source, 0, nil
                                ) as? [CFString: Any],
                            let w = props[
                                kCGImagePropertyPixelWidth
                            ] as? Int,
                            let h = props[
                                kCGImagePropertyPixelHeight
                            ] as? Int
                        else { return false }
                        let ratio = Float(w) / Float(h)
                        return ratio > 1.8 && ratio < 2.2
                    }

                    if isDual {
                        // Direct dual-fisheye → fisheye
                        // skips ~52 MB ERP intermediate
                        let srcPx: (px: [UInt8],
                                     w: Int, h: Int)? =
                            autoreleasepool {
                                DualFisheyeStitcher
                                    .loadPixels(
                                        url: erpURL,
                                        maxWidth: 4096)
                            }
                        guard let src = srcPx
                        else { continue }

                        for isFront in [true, false] {
                            autoreleasepool {
                                let subdir = isFront
                                    ? "fisheye_front"
                                    : "fisheye_back"
                                guard var fisheyePx =
                                    DualFisheyeStitcher
                                        .reprojectToFisheye(
                                            srcPx: src.px,
                                            srcW: src.w,
                                            srcH: src.h,
                                            fisheyeSize: feSize,
                                            fisheyeHalfFovRad:
                                                Self.fisheyeHalfFovRad,
                                            fisheyeRadiusFrac:
                                                Self.fisheyeRadiusFrac,
                                            isFrontLens: isFront,
                                            rollDeg: roll)
                                else { return }

                                if let cp = colorProfile {
                                    ColorProfileCalibrator
                                        .applyInPlace(
                                            cp,
                                            pixels:
                                                &fisheyePx)
                                }

                                let imgDst = imagesDir
                                    .appendingPathComponent(
                                        subdir)
                                    .appendingPathComponent(
                                        "\(scan.scanName)"
                                        + ".jpg")
                                Self.writeJPEGFromPixels(
                                    &fisheyePx,
                                    width: feSize,
                                    height: feSize,
                                    quality: 0.92,
                                    to: imgDst)

                                let mskDst = masksDir
                                    .appendingPathComponent(
                                        subdir)
                                    .appendingPathComponent(
                                        "\(scan.scanName)"
                                        + ".png")
                                if let png = camTiles?[
                                    isFront ? 0 : 1]
                                {
                                    try? png.write(
                                        to: mskDst)
                                }
                            }
                        }
                    } else {
                        // Pre-stitched ERP fallback
                        guard let erp = buildERPPixels()
                        else { continue }
                        for isFront in [true, false] {
                            autoreleasepool {
                                let subdir = isFront
                                    ? "fisheye_front"
                                    : "fisheye_back"
                                guard var result =
                                    Self.reprojectERPtoFisheye(
                                        erpPx: erp.px,
                                        erpW: erp.w,
                                        erpH: erp.h,
                                        maskPx: nil,
                                        fisheyeSize: feSize,
                                        isFrontLens: isFront
                                    ) else { return }

                                let imgDst = imagesDir
                                    .appendingPathComponent(
                                        subdir)
                                    .appendingPathComponent(
                                        "\(scan.scanName)"
                                        + ".jpg")
                                Self.writeJPEGFromPixels(
                                    &result.imagePx,
                                    width: feSize,
                                    height: feSize,
                                    quality: 0.92,
                                    to: imgDst)

                                let mskDst = masksDir
                                    .appendingPathComponent(
                                        subdir)
                                    .appendingPathComponent(
                                        "\(scan.scanName)"
                                        + ".png")
                                if let png = camTiles?[
                                    isFront ? 0 : 1]
                                {
                                    try? png.write(
                                        to: mskDst)
                                } else if let png =
                                    Self.pngDataFromPixels(
                                        &result.maskPx,
                                        width: feSize,
                                        height: feSize)
                                {
                                    try? png.write(
                                        to: mskDst)
                                }
                            }
                        }
                    }
                } else {
                    // Detect dual-fisheye vs pre-stitched
                    let srcInfo: (isDual: Bool,
                                  w: Int, h: Int)? =
                        autoreleasepool {
                            guard let source =
                                CGImageSourceCreateWithURL(
                                    erpURL as CFURL, nil),
                                let props =
                                    CGImageSourceCopyPropertiesAtIndex(
                                        source, 0, nil
                                    ) as? [CFString: Any],
                                let w = props[
                                    kCGImagePropertyPixelWidth
                                ] as? Int,
                                let h = props[
                                    kCGImagePropertyPixelHeight
                                ] as? Int
                            else { return nil }
                            let ratio = Float(w) / Float(h)
                            let isDual = ratio > 1.8
                                && ratio < 2.2
                            return (isDual, w, h)
                        }
                    guard let info = srcInfo
                    else { continue }

                    if info.isDual {
                        // Direct tile from dual-fisheye —
                        // no 134 MB ERP intermediate.
                        // loadPixels uses subsample for
                        // >8000 px sources (~71 MB vs ~283 MB)
                        let srcMax = max(
                            3072,
                            Self.tileSize * 8 / 2 + 1024)
                        let srcPx: (px: [UInt8],
                                    w: Int, h: Int)? =
                            autoreleasepool {
                                DualFisheyeStitcher
                                    .loadPixels(
                                        url: erpURL,
                                        maxWidth: srcMax)
                            }
                        guard let src = srcPx
                        else { continue }

                        let model = cameraConfig.cameras
                            .first { $0.id == camId }?
                            .model ?? ""
                        let roll = DualFisheyeStitcher
                            .rollCorrection(
                                forModel: model)
                        let faceRots =
                            ERPTileSlicer
                                .computeFaceRotations()
                        let ts = slicer.tileSize
                        let fov = slicer.tileFovDeg

                        for faceIdx in 0..<8 {
                            autoreleasepool {
                                let faceDir = imagesDir
                                    .appendingPathComponent(
                                        String(
                                            format:
                                                "face_%02d",
                                            faceIdx))
                                try? fm.createDirectory(
                                    at: faceDir,
                                    withIntermediateDirectories:
                                        true)
                                let dst = faceDir
                                    .appendingPathComponent(
                                        "\(scan.scanName)"
                                        + ".jpg")

                                if var tilePx =
                                    DualFisheyeStitcher
                                        .sliceTileFromDualFisheye(
                                            srcPx: src.px,
                                            srcW: src.w,
                                            srcH: src.h,
                                            tileSize: ts,
                                            faceRotation:
                                                faceRots[
                                                    faceIdx],
                                            tileFovDeg: fov,
                                            rollDeg: roll)
                                {
                                    if let cp =
                                        colorProfile
                                    {
                                        ColorProfileCalibrator
                                            .applyInPlace(
                                                cp,
                                                pixels:
                                                    &tilePx)
                                    }
                                    Self.writeJPEGFromPixels(
                                        &tilePx,
                                        width: ts,
                                        height: ts,
                                        quality: 0.92,
                                        to: dst)
                                }

                                let maskFaceDir = masksDir
                                    .appendingPathComponent(
                                        String(
                                            format:
                                                "face_%02d",
                                            faceIdx))
                                try? fm.createDirectory(
                                    at: maskFaceDir,
                                    withIntermediateDirectories:
                                        true)
                                let mskDst = maskFaceDir
                                    .appendingPathComponent(
                                        "\(scan.scanName)"
                                        + ".png")
                                if let png =
                                    camTiles?[faceIdx]
                                {
                                    try? png.write(
                                        to: mskDst)
                                } else {
                                    Self.writeWhiteMask(
                                        to: mskDst,
                                        width: ts,
                                        height: ts)
                                }
                            }
                        }
                    } else {
                        // Pre-stitched ERP — use remap
                        guard let erp = buildERPPixels()
                        else { continue }
                        for faceIdx in 0..<8 {
                            autoreleasepool {
                                let faceDir = imagesDir
                                    .appendingPathComponent(
                                        String(
                                            format:
                                                "face_%02d",
                                            faceIdx))
                                try? fm.createDirectory(
                                    at: faceDir,
                                    withIntermediateDirectories:
                                        true)
                                let dst = faceDir
                                    .appendingPathComponent(
                                        "\(scan.scanName)"
                                        + ".jpg")
                                if let tile =
                                    slicer.sliceTile(
                                        at: faceIdx,
                                        srcPx: erp.px,
                                        srcW: erp.w,
                                        srcH: erp.h),
                                    let jpegData =
                                        tile.jpegData(
                                            compressionQuality:
                                                0.92)
                                {
                                    try? jpegData.write(
                                        to: dst)
                                }

                                let maskFaceDir = masksDir
                                    .appendingPathComponent(
                                        String(
                                            format:
                                                "face_%02d",
                                            faceIdx))
                                try? fm.createDirectory(
                                    at: maskFaceDir,
                                    withIntermediateDirectories:
                                        true)
                                let mskDst = maskFaceDir
                                    .appendingPathComponent(
                                        "\(scan.scanName)"
                                        + ".png")
                                if let png =
                                    camTiles?[faceIdx]
                                {
                                    try? png.write(
                                        to: mskDst)
                                } else {
                                    Self.writeWhiteMask(
                                        to: mskDst,
                                        width:
                                            slicer.tileSize,
                                        height:
                                            slicer.tileSize)
                                }
                            }
                        }
                    }
                }
            }
        } }
    }

    /// Memory footprint in MB (phys_footprint), the figure iOS compares
    /// with its per-app limit. resident_size also counts file-backed
    /// pages that do not count toward that limit.
    private static func memoryMB() -> Int {
        var info = task_vm_info_data_t()
        var count = mach_msg_type_number_t(
            MemoryLayout<task_vm_info_data_t>
                .size / MemoryLayout<integer_t>.size)
        let result = withUnsafeMutablePointer(
            to: &info
        ) {
            $0.withMemoryRebound(
                to: integer_t.self,
                capacity: Int(count)
            ) {
                task_info(
                    mach_task_self_,
                    task_flavor_t(TASK_VM_INFO),
                    $0, &count)
            }
        }
        guard result == KERN_SUCCESS else { return -1 }
        return Int(info.phys_footprint / (1024 * 1024))
    }

    /// Mean angular rate (deg/s) recorded for each X5 shot of a scan,
    /// read from its pose.json. Empty when none was recorded.
    private func x5MeanRates(scanName: String) -> [String: Double] {
        let url = sessionDirectory.appendingPathComponent(
            "raw/iphone/\(scanName)/pose.json")
        guard let data = try? Data(contentsOf: url),
              let pose = try? JSONDecoder().decode(
                ScanPose.self, from: data)
        else { return [:] }
        var rates: [String: Double] = [:]
        for c in pose.insta360Captures {
            if let m = c.meanAngularRateDps {
                rates[c.cameraId] = m
            }
        }
        return rates
    }

    /// Footprint and the headroom left before iOS ends the app.
    static func memorySummary() -> String {
        let leftMB = os_proc_available_memory() / (1024 * 1024)
        return "\(memoryMB()) MB used, \(leftMB) MB left"
    }

    /// All-white mask (COLMAP: white=valid), width×height pixels.
    private static func writeWhiteMask(
        to url: URL, width: Int, height: Int
    ) {
        autoreleasepool {
            // One pixel per point, 8 bits per channel. The default
            // format uses the screen scale (3x) and can pick a 16-bit
            // wide-colour buffer, so each mask came out 3x the size of
            // its image and took 100 MB or more to render.
            let format = UIGraphicsImageRendererFormat()
            format.scale = 1
            format.preferredRange = .standard
            let renderer = UIGraphicsImageRenderer(
                size: CGSize(width: width, height: height),
                format: format
            )
            let img = renderer.image { _ in
                UIColor.white.setFill()
                UIRectFill(CGRect(x: 0, y: 0, width: width, height: height))
            }
            if let data = img.pngData() {
                try? data.write(to: url)
            }
        }
    }

    private static func writeJPEGFromPixels(
        _ px: inout [UInt8],
        width: Int, height: Int,
        quality: CGFloat, to url: URL
    ) {
        guard let ctx = CGContext(
            data: &px, width: width, height: height,
            bitsPerComponent: 8,
            bytesPerRow: width * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo
                .noneSkipLast.rawValue
        ), let cg = ctx.makeImage() else { return }
        let data = NSMutableData()
        guard let dest =
            CGImageDestinationCreateWithData(
                data,
                "public.jpeg" as CFString, 1, nil
            )
        else { return }
        CGImageDestinationAddImage(
            dest, cg,
            [kCGImageDestinationLossyCompressionQuality:
                quality] as CFDictionary
        )
        guard CGImageDestinationFinalize(dest)
        else { return }
        try? (data as Data).write(to: url)
    }

    private static func pngDataFromPixels(
        _ px: inout [UInt8],
        width: Int, height: Int
    ) -> Data? {
        guard let ctx = CGContext(
            data: &px, width: width, height: height,
            bitsPerComponent: 8,
            bytesPerRow: width * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo
                .noneSkipLast.rawValue
        ), let cg = ctx.makeImage() else { return nil }
        let data = NSMutableData()
        guard let dest =
            CGImageDestinationCreateWithData(
                data,
                "public.png" as CFString, 1, nil
            )
        else { return nil }
        CGImageDestinationAddImage(dest, cg, nil)
        guard CGImageDestinationFinalize(dest)
        else { return nil }
        return data as Data
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

    private func loadAllCameraMaskURLs() -> [String: URL] {
        guard let dir = FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first?.appendingPathComponent(
            "atlas_sessions/masks")
        else { return [:] }
        var result: [String: URL] = [:]
        if let files = try? FileManager.default
            .contentsOfDirectory(
                at: dir,
                includingPropertiesForKeys: nil)
        {
            for file in files
            where file.pathExtension == "png" {
                let camId = file
                    .deletingPathExtension()
                    .lastPathComponent
                result[camId] = file
            }
        }
        return result
    }

    private static func loadMaskPixels(
        url: URL, maxWidth: Int
    ) -> (px: [UInt8], w: Int, h: Int)? {
        guard let source =
            CGImageSourceCreateWithURL(
                url as CFURL, nil),
            let props =
                CGImageSourceCopyPropertiesAtIndex(
                    source, 0, nil
                ) as? [CFString: Any],
            let srcW = props[
                kCGImagePropertyPixelWidth
            ] as? Int
        else { return nil }

        var opts: [CFString: Any] = [
            kCGImageSourceThumbnailMaxPixelSize:
                maxWidth,
            kCGImageSourceCreateThumbnailFromImageAlways:
                true,
            kCGImageSourceShouldCacheImmediately:
                true,
            kCGImageSourceShouldCache:
                false,
        ]
        if srcW > 8000 {
            opts[kCGImageSourceSubsampleFactor] = 2
        }
        guard let cg =
            CGImageSourceCreateThumbnailAtIndex(
                source, 0,
                opts as CFDictionary)
        else { return nil }

        let w = cg.width, h = cg.height
        guard w > 0, h > 0 else { return nil }

        // Invert mask (white = masked → black) and
        // extract top-down RGBA pixels in one pass.
        // Draw with the identity CTM: it already puts the
        // image's top row first, the layout ERPTileSlicer
        // and reprojectERPtoFisheye read (as in
        // DualFisheyeStitcher.extractPixels). A flipped CTM
        // here loaded the mask upside down.
        var px = [UInt8](
            repeating: 0, count: w * h * 4)
        guard let ctx = CGContext(
            data: &px, width: w, height: h,
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
                width: w, height: h))

        for i in stride(from: 0, to: px.count,
                         by: 4) {
            px[i] = 255 - px[i]
            px[i + 1] = 255 - px[i + 1]
            px[i + 2] = 255 - px[i + 2]
        }

        return (px, w, h)
    }

    // MARK: - ERP → Fisheye reprojection

    /// Reprojects an equirectangular image to an equidistant fisheye image.
    /// Returns the fisheye image and its circular mask.
    private static func extractERPPixels(
        _ image: UIImage
    ) -> (px: [UInt8], w: Int, h: Int)? {
        guard let cg = image.cgImage else { return nil }
        let w = cg.width, h = cg.height
        guard w > 0, h > 0 else { return nil }
        var px = [UInt8](
            repeating: 0, count: w * h * 4
        )
        guard let ctx = CGContext(
            data: &px, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo
                .noneSkipLast.rawValue
        ) else { return nil }
        ctx.draw(
            cg,
            in: CGRect(x: 0, y: 0, width: w, height: h)
        )
        return (px, w, h)
    }

    private static func reprojectERPtoFisheye(
        erpPx: [UInt8], erpW: Int, erpH: Int,
        maskPx: [UInt8]?,
        fisheyeSize: Int,
        isFrontLens: Bool
    ) -> (imagePx: [UInt8], maskPx: [UInt8])? {
        guard erpW > 0, erpH > 0 else { return nil }

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
                        let eu: Float = isFrontLens
                            ? erpWf * 0.5 : 0
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

                let bx = xSign * sinT * dx / r
                let by = sinT * dy / r
                let bz = zSign * cosT

                let lat = asinf(max(-1, min(1, -by)))
                let lon = atan2f(bx, bz)

                let eu = (lon / (2 * .pi) + 0.5) * erpWf
                let ev = (0.5 - lat / .pi) * erpHf

                let eix = min(max(Int(eu), 0), erpW - 2)
                let eiy = min(max(Int(ev), 0), erpH - 2)
                let fx = eu - Float(eix)
                let fy = ev - Float(eiy)
                let w00 = (1 - fx) * (1 - fy)
                let w10 = fx * (1 - fy)
                let w01 = (1 - fx) * fy
                let w11 = fx * fy
                let base = eiy * erpStride + eix * 4

                var rv: Float = 0, gv: Float = 0
                var bv: Float = 0
                for (off, wt) in [
                    (base, w00),
                    (base + 4, w10),
                    (base + erpStride, w01),
                    (base + erpStride + 4, w11),
                ] {
                    rv += Float(erpPx[off]) * wt
                    gv += Float(erpPx[off + 1]) * wt
                    bv += Float(erpPx[off + 2]) * wt
                }
                outPx[idx] = UInt8(clamping: Int(rv))
                outPx[idx + 1] = UInt8(clamping: Int(gv))
                outPx[idx + 2] = UInt8(clamping: Int(bv))
                outPx[idx + 3] = 255

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

        return (outPx, outMask)
    }

    // MARK: - Blur detection

    static func blurScore(
        imageURL: URL
    ) -> Float? {
        guard let source =
            CGImageSourceCreateWithURL(
                imageURL as CFURL, nil),
            let cgImage =
                CGImageSourceCreateThumbnailAtIndex(
                    source, 0, [
                        kCGImageSourceThumbnailMaxPixelSize:
                            640,
                        kCGImageSourceCreateThumbnailFromImageAlways:
                            true,
                        kCGImageSourceShouldCacheImmediately:
                            true,
                        kCGImageSourceShouldCache:
                            false,
                        kCGImageSourceSubsampleFactor: 4,
                    ] as CFDictionary)
        else { return nil }

        let srcW = cgImage.width
        let srcH = cgImage.height
        guard srcW > 0, srcH > 0 else { return nil }

        let targetW = 320
        let targetH = Int(
            Float(targetW) * Float(srcH)
                / Float(srcW)
        )
        guard targetW > 2, targetH > 2
        else { return nil }

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
        cameraExportMode: CameraExportMode = .pinholeTiles,
        trajectory: [(timestamp: Double, pose: simd_float4x4)]? = nil
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
            cameraExportMode: cameraExportMode,
            trajectory: trajectory
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
        cameraExportMode: CameraExportMode,
        trajectory: [(timestamp: Double, pose: simd_float4x4)]? = nil
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
                let x5Pose: simd_float4x4
                if let cmdTs = scan.insta360CommandTimestamp,
                   let traj = trajectory,
                   let interpolated = Self.interpolateTrajectory(
                       traj, at: cmdTs + Self.x5ShutterDelay
                   )
                {
                    x5Pose = interpolated
                } else {
                    x5Pose = T_wc
                }

                for (camIdx, camCfg) in cameraConfig.cameras.enumerated() {
                    let T_insta_arkit = Self.erpFromARKit(
                        camCfg.extrinsic
                    )
                    let T_insta_world =
                        T_insta_arkit * x5Pose.inverse
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

    // MARK: - Trajectory interpolation

    private func loadTrajectory()
        -> [(timestamp: Double, pose: simd_float4x4)]?
    {
        Self.loadTrajectory(sessionDirectory: sessionDirectory)
    }

    static func loadTrajectory(sessionDirectory: URL)
        -> [(timestamp: Double, pose: simd_float4x4)]?
    {
        let url = sessionDirectory
            .appendingPathComponent("raw/trajectory.json")
        guard let data = try? Data(contentsOf: url),
              let traj = try? JSONDecoder().decode(
                  TrajectoryData.self, from: data
              )
        else { return nil }
        return traj.poses.compactMap { entry in
            let T = entry.transform
            guard T.count == 4,
                  T.allSatisfy({ $0.count == 4 })
            else { return nil }
            let pose = simd_float4x4(columns: (
                SIMD4(T[0][0], T[0][1], T[0][2], T[0][3]),
                SIMD4(T[1][0], T[1][1], T[1][2], T[1][3]),
                SIMD4(T[2][0], T[2][1], T[2][2], T[2][3]),
                SIMD4(T[3][0], T[3][1], T[3][2], T[3][3])
            ))
            return (timestamp: entry.timestamp, pose: pose)
        }
    }

    static func interpolateTrajectory(
        _ trajectory: [(timestamp: Double, pose: simd_float4x4)],
        at timestamp: Double
    ) -> simd_float4x4? {
        guard trajectory.count >= 2
        else { return trajectory.first?.pose }
        guard let afterIdx = trajectory.firstIndex(
            where: { $0.timestamp >= timestamp }
        ) else { return trajectory.last?.pose }
        guard afterIdx > 0
        else { return trajectory.first?.pose }
        let before = trajectory[afterIdx - 1]
        let after = trajectory[afterIdx]
        let t = Float(
            (timestamp - before.timestamp)
                / (after.timestamp - before.timestamp)
        )
        return PoseInterpolation.interpolate(
            from: before.pose, to: after.pose, t: t
        )
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
    let insta360CommandTimestamp: Double?  // systemUptime when X5 capture was issued
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

// MARK: - ARKit tracking check

/// Flags ARKit position jumps (tracking corrections) and scans taken
/// while tracking was limited, so a session whose ARKit poses need
/// replacing by SfM can be spotted on the phone.
enum ARKitTrackingCheck {
    struct Jump: Codable {
        /// Seconds since the first trajectory pose.
        let time: Double
        let jumpCm: Float
        let nearestScan: String
    }

    struct LimitedScan: Codable {
        let scan: String
        let state: String
    }

    struct Result: Codable {
        let jumps: [Jump]
        let limitedScans: [LimitedScan]
        let summary: String

        var hasIssues: Bool {
            !jumps.isEmpty || !limitedScans.isEmpty
        }
    }

    /// A step over 5 cm between consecutive 60 Hz poses (over 3 m/s)
    /// is a tracking correction, not walking: in a 9-minute walk the
    /// 99.9th-percentile step was 2.4 cm.
    static let minJumpM: Float = 0.05
    static let minJumpSpeed: Float = 3.0

    static func run(
        sessionDirectory: URL,
        trajectory: [(timestamp: Double, pose: simd_float4x4)]?
    ) -> Result {
        // Scan times and tracking states come from each pose.json.
        var scanTimes: [(t: Double, name: String)] = []
        var limited: [LimitedScan] = []
        let iphoneDir = sessionDirectory
            .appendingPathComponent("raw/iphone")
        let dirs = ((try? FileManager.default.contentsOfDirectory(
            at: iphoneDir, includingPropertiesForKeys: nil)) ?? [])
            .filter { $0.lastPathComponent.hasPrefix("scan_") }
            .sorted { $0.path < $1.path }
        for dir in dirs {
            guard let data = try? Data(contentsOf:
                    dir.appendingPathComponent("pose.json")),
                  let pose = try? JSONDecoder().decode(
                    ScanPose.self, from: data)
            else { continue }
            let name = dir.lastPathComponent
            scanTimes.append((t: pose.timestamp, name: name))
            if let state = pose.trackingState, state != "normal" {
                limited.append(LimitedScan(scan: name, state: state))
            }
        }

        var jumps: [Jump] = []
        if let traj = trajectory, traj.count >= 2 {
            let t0 = traj[0].timestamp
            for i in 1..<traj.count {
                let dt = Float(traj[i].timestamp
                    - traj[i - 1].timestamp)
                guard dt > 0 else { continue }
                let a = traj[i - 1].pose.columns.3
                let b = traj[i].pose.columns.3
                let step = simd_length(SIMD3<Float>(
                    b.x - a.x, b.y - a.y, b.z - a.z))
                guard step > minJumpM, step / dt > minJumpSpeed
                else { continue }
                let t = traj[i].timestamp
                let nearest = scanTimes.min {
                    abs($0.t - t) < abs($1.t - t)
                }?.name ?? "?"
                jumps.append(Jump(
                    time: t - t0, jumpCm: step * 100,
                    nearestScan: nearest))
            }
        }

        var parts: [String] = []
        if !jumps.isEmpty {
            let maxCm = Int((jumps.map(\.jumpCm).max() ?? 0).rounded())
            var seen = Set<String>()
            let scans = jumps.map(\.nearestScan)
                .filter { seen.insert($0).inserted }
            let list = scans.prefix(8).joined(separator: ", ")
                + (scans.count > 8 ? ", …" : "")
            parts.append("\(jumps.count) position jumps"
                + " (up to \(maxCm) cm) near \(list)")
        }
        if !limited.isEmpty {
            let list = limited.prefix(8).map(\.scan)
                .joined(separator: ", ")
                + (limited.count > 8 ? ", …" : "")
            parts.append("tracking limited at \(limited.count)"
                + " scans: \(list)")
        }
        let summary = parts.isEmpty
            ? "No ARKit position jumps found"
            : parts.joined(separator: "; ")
        return Result(
            jumps: jumps, limitedScans: limited, summary: summary)
    }

    /// Writes the result to <session>/arkit_check.json for the
    /// processing pipeline.
    static func save(_ result: Result, in sessionDirectory: URL) {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        encoder.keyEncodingStrategy = .convertToSnakeCase
        if let data = try? encoder.encode(result) {
            try? data.write(to: sessionDirectory
                .appendingPathComponent("arkit_check.json"))
        }
    }
}

// MARK: - Export trace

/// Progress lines of the current export, written to
/// <session>/export_trace.log and flushed to disk one by one. iOS leaves
/// no crash report for some ways the app can end, so the last line
/// still says which step the export was in. The export screen shows
/// the last lines when an export did not finish.
enum ExportTrace {
    static let fileName = "export_trace.log"
    private static let endMarker = "export ended: "
    private static let lock = NSLock()
    private static var handle: FileHandle?
    private static var start = Date()

    /// Starts the trace of a new export, replacing the previous one, and
    /// starts writing crash details (signal, stack, uncaught exceptions)
    /// into it.
    static func begin(sessionDirectory: URL, _ line: String) {
        lock.lock()
        defer { lock.unlock() }
        RigCalibratorBridge.stopCrashCapture()
        try? handle?.close()
        let url = sessionDirectory.appendingPathComponent(fileName)
        FileManager.default.createFile(atPath: url.path, contents: nil)
        handle = try? FileHandle(forWritingTo: url)
        start = Date()
        write(line)
        if let fd = handle?.fileDescriptor {
            let env = RigCalibratorBridge.installCrashCapture(fileDescriptor: fd)
            for envLine in env.split(separator: "\n") {
                write(String(envLine))
            }
        }
    }

    /// Adds a line while an export is running; does nothing otherwise.
    static func note(_ line: String) {
        lock.lock()
        defer { lock.unlock() }
        write(line)
    }

    /// Writes the last line of the export and closes the trace.
    static func finish(_ line: String) {
        lock.lock()
        defer { lock.unlock() }
        write(endMarker + line)
        RigCalibratorBridge.stopCrashCapture()
        try? handle?.close()
        handle = nil
    }

    /// Lines of the session's trace when its export did not finish, i.e.
    /// the app ended during it; nil otherwise. When a crash was captured
    /// ("!!! " lines), the `count` lines before it and its first lines;
    /// otherwise the last `count` lines.
    static func unfinished(
        sessionDirectory: URL, count: Int = 4
    ) -> [String]? {
        let url = sessionDirectory.appendingPathComponent(fileName)
        guard let text = try? String(contentsOf: url, encoding: .utf8)
        else { return nil }
        let lines = text.split(separator: "\n").map(String.init)
        guard let last = lines.last, !last.contains(endMarker)
        else { return nil }
        if let crash = lines.firstIndex(where: { $0.hasPrefix("!!! ") }) {
            let lo = max(0, crash - count)
            let hi = min(lines.count, crash + 8)
            return Array(lines[lo..<hi])
        }
        return Array(lines.suffix(count))
    }

    /// Caller holds `lock`. Each line gets the elapsed time and, unless
    /// it already has them, the memory numbers, so a memory kill shows
    /// up as falling headroom.
    private static func write(_ line: String) {
        guard let h = handle else { return }
        let t = String(format: "%6.1fs ",
                       Date().timeIntervalSince(start))
        let mem = line.contains("MB left")
            ? "" : " (\(COLMAPExporter.memorySummary()))"
        try? h.write(contentsOf: Data((t + line + mem + "\n").utf8))
        try? h.synchronize()
    }
}
