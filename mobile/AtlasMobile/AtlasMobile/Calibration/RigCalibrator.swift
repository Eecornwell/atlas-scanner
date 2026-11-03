import Foundation
import simd
import Accelerate
import CoreGraphics
import ImageIO

/// On-device rig calibration replacing calibrate_rig.py.
///
/// Triangulates 3D points from iPhone ARKit poses, PnP-solves each
/// X5 tile, then fits a single rotation + centre correction for the
/// rig. The correction is expressed in the sensor_from_rig frame,
/// matching the Python script's convention.
final class RigCalibrator {

    enum Status: String { case applied, flagged, skipped }

    struct Result {
        let status: Status
        let reason: String
        let correctionAngleDeg: Float
        let correctionShiftCm: Float
        let errorBeforeDeg: Float
        let errorAfterDeg: Float
        let facesUsed: Int
        let facesTotal: Int
        /// Rotation correction: S_new = S_old * Q.
        let correctionQ: simd_float3x3
        /// Centre shift in rig frame (metres).
        let correctionD: SIMD3<Float>
        /// For COLMAPExporter.setRigCorrection().
        let rigCorrectionForExport: simd_float3x3
    }

    struct Config {
        var maxWidth: Int = 512
        var tileExtractSize: Int = 512
        var tileFovDeg: Float = 65.0
        var matchRatio: Float = 0.75
        var matchWindowIphone: Int = 3
        var matchNearestPerTile: Int = 2
        var triangMinViews: Int = 3
        var triangMaxReproj: Float = 4.0
        var pnpMinInliers: Int = 20
        var minUnmasked: Float = 0.30
        var minFaces: Int = 8
        var maxResidualDeg: Float = 2.0
        var maxCorrectionDeg: Float = 30.0
        var maxCorrectionCm: Float = 25.0
        var x5ShutterDelay: Double = 1.0
        /// Number of X5 scans used, evenly spaced over the scans that
        /// have an X5 image. X5 images are processed one at a time,
        /// so this sets run time rather than peak memory.
        var maxCalibScans: Int = 8
        /// iPhone images used around each X5 scan: that scan plus this
        /// many scans either side, so 3D points are seen from >= 3
        /// nearby iPhone positions.
        var iphoneNeighbors: Int = 2
        /// Width of the dual-fisheye source the tiles are cut from.
        /// 3072 px gives ~7 px/deg in each lens, about the same as a
        /// 512 px / 65° tile, so tiles are not cut from a blurred,
        /// ~3x upsampled source (1024 px).
        var srcLoadMaxWidth: Int = 3072
        /// OpenCV SIFT features kept per image (the strongest ones).
        var maxKeypoints: Int = 2000
    }

    private struct TileId: Hashable {
        let scanIndex: Int
        let faceIndex: Int
    }

    /// Maps an image ID to its scan index (for iPhone images).
    private struct IphoneEntry {
        let imgId: Int
        let scanIndex: Int
    }

    /// OpenCV SIFT keypoints and descriptors per image ID, held by
    /// RigSIFTFeatures (RigCalibratorBridge.mm) until reset().
    private final class FeatureStore {
        typealias PixelPair = (qx: Float, qy: Float, tx: Float, ty: Float)

        private let sift: RigSIFTFeatures
        /// Image pairs with fewer ratio-test matches are dropped.
        private let minMatches = 8

        init(maxKeypoints: Int) {
            sift = RigSIFTFeatures(
                maxFeatures: Int32(clamping: maxKeypoints))
        }

        /// Loads an image, scales its long side to at most `maxWidth`
        /// (the scale buildPortraitProjMats assumes) and extracts
        /// features. Returns the keypoint count, or -1 on failure.
        func addImage(url: URL, imageId: Int, maxWidth: Int) -> Int {
            guard let g = Self.loadGray(url: url, maxWidth: maxWidth)
            else { return -1 }
            return addGray(
                g.px, width: g.w, height: g.h, imageId: imageId)
        }

        /// Extracts features from a width×height grayscale buffer.
        /// Returns the feature count, or -1 on failure.
        func addGray(
            _ gray: [UInt8], width: Int, height: Int, imageId: Int
        ) -> Int {
            guard width >= 64, height >= 64,
                  gray.count >= width * height
            else { return -1 }
            return Int(sift.add(
                gray: gray,
                width: Int32(width), height: Int32(height),
                imageId: Int32(imageId)))
        }

        /// Ratio-test matches as (query x/y, train x/y) pixel pairs,
        /// or nil when fewer than `minMatches` survive.
        func match(
            _ queryId: Int, _ trainId: Int, ratio: Float
        ) -> [PixelPair]? {
            guard let data = sift.match(
                    query: Int32(queryId), train: Int32(trainId),
                    ratio: ratio)
            else { return nil }
            let n = data.count / (4 * MemoryLayout<Float>.size)
            guard n >= minMatches else { return nil }
            var pairs: [PixelPair] = []
            pairs.reserveCapacity(n)
            let size = MemoryLayout<Float>.size
            data.withUnsafeBytes { (raw: UnsafeRawBufferPointer) in
                for i in 0..<n {
                    let o = 4 * size * i
                    pairs.append((
                        qx: raw.load(fromByteOffset: o, as: Float.self),
                        qy: raw.load(fromByteOffset: o + size, as: Float.self),
                        tx: raw.load(fromByteOffset: o + 2 * size, as: Float.self),
                        ty: raw.load(fromByteOffset: o + 3 * size, as: Float.self)))
                }
            }
            return pairs
        }

        func reset() {
            sift.reset()
        }

        /// Grayscale copy of an image file with its long side scaled to
        /// at most `maxWidth`, using the same size math as the ObjC
        /// loader it replaces: Int(w * s) × Int(h * s).
        private static func loadGray(
            url: URL, maxWidth: Int
        ) -> (px: [UInt8], w: Int, h: Int)? {
            autoreleasepool { () -> (px: [UInt8], w: Int, h: Int)? in
                guard let source = CGImageSourceCreateWithURL(
                        url as CFURL, nil),
                      let cg = CGImageSourceCreateImageAtIndex(
                        source, 0, nil)
                else { return nil }
                let w = cg.width, h = cg.height
                guard w > 0, h > 0 else { return nil }
                var tw = w, th = h
                let longSide = max(w, h)
                if maxWidth > 0, longSide > maxWidth {
                    let s = Float(maxWidth) / Float(longSide)
                    tw = Int(Float(w) * s)
                    th = Int(Float(h) * s)
                }
                guard tw > 0, th > 0 else { return nil }
                var gray = [UInt8](repeating: 0, count: tw * th)
                let drawn = gray.withUnsafeMutableBytes {
                    buf -> Bool in
                    guard let ctx = CGContext(
                        data: buf.baseAddress,
                        width: tw, height: th,
                        bitsPerComponent: 8, bytesPerRow: tw,
                        space: CGColorSpaceCreateDeviceGray(),
                        bitmapInfo: CGImageAlphaInfo.none.rawValue)
                    else { return false }
                    ctx.interpolationQuality = .high
                    ctx.draw(cg, in: CGRect(
                        x: 0, y: 0, width: tw, height: th))
                    return true
                }
                return drawn ? (px: gray, w: tw, h: th) : nil
            }
        }
    }

    // MARK: - Public entry point

    static func calibrate(
        scans: [ScanExportData],
        cameraConfig: MultiCameraConfig,
        trajectory: [(timestamp: Double, pose: simd_float4x4)]?,
        maskDir: URL?,
        config: Config = Config()
    ) -> Result {
        guard let camCfg = cameraConfig.cameras.first,
              camCfg.enabled
        else { return skip("no enabled Insta360 camera") }

        let scansWithX5 = scans.filter {
            !$0.insta360ImageURLs.isEmpty
        }
        guard scansWithX5.count >= 5 else {
            return skip("fewer than 5 scans with X5 images")
        }

        // The OpenCV calls below must reach the app's own OpenCV 5.
        // INSCoreMedia (Insta360 SDK) exports an OpenCV 4.7 with a
        // different cv::Mat layout; functions bound to it end the app
        // with a signal that try/catch cannot stop. project.yml lists
        // opencv2 first so the linker binds them to OpenCV 5.
        log("OpenCV functions linked from: "
            + RigCalibratorBridge.openCVBindingReport())
        guard RigCalibratorBridge.openCVLinkedFromApp() else {
            return skip("OpenCV is linked from the Insta360 SDK; "
                + "rebuild with opencv2 first in project.yml")
        }

        // Give this thread its own signal stack, so the export trace still
        // gets a crash report if OpenCV runs it out of stack.
        RigCalibratorBridge.useAlternateSignalStackOnCurrentThread()

        // OpenCV SIFT features. The pure-Swift FAST/BRIEF detector used
        // while OpenCV crashed found about a third as many X5 tile ↔
        // iPhone matches, too few to solve 8 tiles.
        let features = FeatureStore(maxKeypoints: config.maxKeypoints)

        // Pick the X5 scans from the scans that have an X5 image. The
        // old code spaced its picks over all scans, so when only some
        // scans had an X5 image it could pick none ("loaded 0/0").
        // Indices below are positions in `scans`.
        let x5Indices = scans.indices.filter {
            !scans[$0].insta360ImageURLs.isEmpty
        }
        let maxPicks = max(2, config.maxCalibScans)
        let x5Picks: [Int]
        if x5Indices.count > maxPicks {
            let step = Float(x5Indices.count - 1)
                / Float(maxPicks - 1)
            x5Picks = (0..<maxPicks).map {
                x5Indices[Int(Float($0) * step + 0.5)]
            }
        } else {
            x5Picks = x5Indices
        }

        // iPhone images: each picked scan plus its neighbours, which
        // overlap it and give the >= 3 views triangulation needs.
        let reach = max(0, config.iphoneNeighbors)
        var iphoneIdx = Set<Int>()
        for k in x5Picks {
            let lo = max(0, k - reach)
            let hi = min(scans.count - 1, k + reach)
            for j in lo...hi { iphoneIdx.insert(j) }
        }
        let calibIndices = iphoneIdx.sorted()
        let calibScans = calibIndices.map { scans[$0] }
        log("using \(x5Picks.count) of \(x5Indices.count) X5 scans"
            + " and \(calibIndices.count) iPhone images")

        log("begin calibration: \(scansWithX5.count)"
            + " X5 scans, \(scans.count) total")

        let faceRots = ERPTileSlicer.computeFaceRotations()
        let nFaces = faceRots.count
        let tileF = Float(config.tileExtractSize)
            / (2.0 * tan(
                config.tileFovDeg * .pi / 360.0))
        let tileCx = Float(config.tileExtractSize) / 2
        let tileCy = Float(config.tileExtractSize) / 2

        // ── Phase 1: feature extraction ───────────────────

        log("extracting features…")
        var nextId = 0
        var iphoneEntries: [IphoneEntry] = []
        var tileIds: [TileId: Int] = [:]
        var iphoneFeatures = 0
        var tileFeatures = 0

        for (ci, scan) in calibScans.enumerated() {
            autoreleasepool {
                let si = calibIndices[ci]
                guard let url = scan.iphoneImageURL
                else { return }
                let id = nextId; nextId += 1
                let n = features.addImage(
                    url: url, imageId: id,
                    maxWidth: config.maxWidth)
                if n > 0 {
                    iphoneEntries.append(
                        IphoneEntry(
                            imgId: id, scanIndex: si))
                    iphoneFeatures += n
                }
            }
        }
        guard iphoneEntries.count >= 5 else {
            return skip("fewer than 5 iPhone images")
        }
        log("iPhones done, \(iphoneFeatures / iphoneEntries.count)"
            + " features per image")

        let calibX5 = x5Picks
        let rollDeg = DualFisheyeStitcher.rollCorrection(
            forModel: camCfg.model)

        // Tiles are cut at the feature resolution. The old bridge cut at
        // tileExtractSize and then downscaled to maxWidth; tileScale in
        // phase 5 assumes this same size.
        let tileSide = min(config.tileExtractSize, config.maxWidth)

        var facesTotal = 0
        var x5Loaded = 0
        for (xi, si) in calibX5.enumerated() {
            let scan = scans[si]
            guard let srcURL = scan.insta360ImageURLs
                .values.first else { continue }
            log("scan \(xi+1)/\(calibX5.count): loading"
                + " \(scan.scanName) 360 image")

            guard let src = autoreleasepool(invoking: {
                DualFisheyeStitcher.loadPixels(
                    url: srcURL,
                    maxWidth: config.srcLoadMaxWidth)
            }) else {
                log("scan \(xi+1)/\(calibX5.count):"
                    + " could not load \(srcURL.lastPathComponent)")
                continue
            }
            x5Loaded += 1

            for fi in 0..<nFaces {
                autoreleasepool {
                    guard let tilePx = DualFisheyeStitcher
                        .sliceTileFromDualFisheye(
                            srcPx: src.px,
                            srcW: src.w, srcH: src.h,
                            tileSize: tileSide,
                            faceRotation: faceRots[fi],
                            tileFovDeg: config.tileFovDeg,
                            rollDeg: rollDeg)
                    else { return }

                    let id = nextId; nextId += 1
                    let n = features.addGray(
                        rgbaToGray(tilePx),
                        width: tileSide, height: tileSide,
                        imageId: id)
                    if n > 0 {
                        tileIds[TileId(
                            scanIndex: si,
                            faceIndex: fi)] = id
                        facesTotal += 1
                        tileFeatures += n
                    }
                }
            }
            log("scan \(xi+1)/\(calibX5.count)"
                + " tiles=\(facesTotal)")
        }
        log("\(iphoneEntries.count) iPhones,"
            + " \(facesTotal) tiles,"
            + " \(tileFeatures / max(1, facesTotal)) features per tile")

        // ── Phase 2: Matching ─────────────────────────────
        log("phase 2: matching")

        typealias Match4 = (qx: Float, qy: Float,
                            tx: Float, ty: Float)
        struct MatchBatch {
            let queryImgId: Int
            let trainImgId: Int
            let pairs: [Match4]
        }
        var batches: [MatchBatch] = []

        // iPhone–iPhone sequential
        for i in 0..<iphoneEntries.count {
            let end = min(i + config.matchWindowIphone,
                          iphoneEntries.count)
            for j in (i + 1)..<end {
                if let pairs = features.match(
                    iphoneEntries[i].imgId,
                    iphoneEntries[j].imgId,
                    ratio: config.matchRatio)
                {
                    batches.append(MatchBatch(
                        queryImgId:
                            iphoneEntries[i].imgId,
                        trainImgId:
                            iphoneEntries[j].imgId,
                        pairs: pairs))
                }
            }
        }

        // Tile → nearest iPhones
        let iphDirs = iphoneEntries.map { e -> SIMD3<Float> in
            let T = scans[e.scanIndex].arkitPose
            return normalize(-SIMD3(
                T.columns.2.x, T.columns.2.y, T.columns.2.z))
        }
        var tileMatches = 0
        for (tid, tileImgId) in tileIds {
            let tDir = tileViewDir(
                scan: scans[tid.scanIndex],
                face: tid.faceIndex,
                faceRots: faceRots, cam: camCfg,
                traj: trajectory,
                delay: config.x5ShutterDelay)
            // Only iPhone images taken near this X5 scan can see the
            // same surfaces; images elsewhere with a similar view
            // direction would only add false matches.
            let nearest = iphDirs.enumerated()
                .filter {
                    abs(iphoneEntries[$0.offset].scanIndex
                        - tid.scanIndex) <= reach
                }
                .sorted { simd_dot(tDir, $0.element)
                        > simd_dot(tDir, $1.element) }
                .prefix(config.matchNearestPerTile)
            for (idx, _) in nearest {
                let iphId = iphoneEntries[idx].imgId
                if let pairs = features.match(
                    tileImgId, iphId,
                    ratio: config.matchRatio)
                {
                    batches.append(MatchBatch(
                        queryImgId: tileImgId,
                        trainImgId: iphId,
                        pairs: pairs))
                    tileMatches += pairs.count
                }
            }
        }
        log("\(batches.count) match batches,"
            + " \(batches.reduce(0) { $0 + $1.pairs.count }) matches"
            + " (\(tileMatches) tile ↔ iPhone)")
        features.reset()

        // ── Phase 3: Track building (union-find) ──────────
        log("phase 3: track building")

        var uf = UnionFind()
        struct PixObs { let imgId: Int; let x: Float; let y: Float }
        var obsTable: [Int64: PixObs] = [:]

        for b in batches {
            for p in b.pairs {
                let qk = obsKey(b.queryImgId, p.qx, p.qy)
                let tk = obsKey(b.trainImgId, p.tx, p.ty)
                uf.unite(qk, tk)
                obsTable[qk] = PixObs(
                    imgId: b.queryImgId, x: p.qx, y: p.qy)
                obsTable[tk] = PixObs(
                    imgId: b.trainImgId, x: p.tx, y: p.ty)
            }
        }

        // Group by root; keep one obs per image
        var tracks: [Int64: [Int: PixObs]] = [:]
        for (key, obs) in obsTable {
            let root = uf.find(key)
            tracks[root, default: [:]][obs.imgId] = obs
        }

        // ── Phase 4: Triangulate from iPhone views ────────
        // OpenCV SVD per track, then PnP below.
        log("phase 4: triangulation of \(tracks.count) tracks")

        let iphoneSet = Set(iphoneEntries.map(\.imgId))
        let projMats = buildPortraitProjMats(
            scans: scans, entries: iphoneEntries,
            maxWidth: config.maxWidth)

        struct Pt3D {
            let pos: SIMD3<Double>
            let tileObs: [Int: PixObs]
        }
        var pts3D: [Pt3D] = []

        var triCalls = 0
        for (_, group) in tracks {
            let iphObs = group.filter { iphoneSet.contains($0.key) }
            let tilObs = group.filter { !iphoneSet.contains($0.key) }
            guard iphObs.count >= config.triangMinViews,
                  !tilObs.isEmpty
            else { continue }

            let arr: [[NSNumber]] = iphObs.map { (id, o) in
                [NSNumber(value: id),
                 NSNumber(value: o.x),
                 NSNumber(value: o.y)]
            }
            triCalls += 1
            if triCalls <= 3 || triCalls % 250 == 0 {
                log("triangulation call \(triCalls), \(arr.count) views")
            }
            if let r = RigCalibratorBridge.triangulatePoint(
                arr, projMatrices: projMats,
                maxReproj: config.triangMaxReproj,
                minObserved: Int32(config.triangMinViews))
            {
                pts3D.append(Pt3D(
                    pos: SIMD3(r[0].doubleValue,
                               r[1].doubleValue,
                               r[2].doubleValue),
                    tileObs: tilObs))
            }
        }
        log("triangulated \(pts3D.count) 3D points in \(triCalls) calls")

        // ── Phase 5: PnP per tile ────────────────────────
        log("phase 5: PnP")

        let imgToTile = Dictionary(uniqueKeysWithValues:
            tileIds.map { ($0.value, $0.key) })
        var tileCorr: [TileId: (
            pts3D: [[NSNumber]], pts2D: [[NSNumber]]
        )] = [:]
        for pt in pts3D {
            for (imgId, obs) in pt.tileObs {
                guard let tid = imgToTile[imgId] else { continue }
                var e = tileCorr[tid] ?? ([], [])
                e.pts3D.append([
                    NSNumber(value: pt.pos.x),
                    NSNumber(value: pt.pos.y),
                    NSNumber(value: pt.pos.z)])
                e.pts2D.append([
                    NSNumber(value: obs.x),
                    NSNumber(value: obs.y)])
                tileCorr[tid] = e
            }
        }

        let tileScale = Float(min(config.maxWidth,
            config.tileExtractSize))
            / Float(config.tileExtractSize)
        let fS = Double(tileF * tileScale)
        let cxS = Double(tileCx * tileScale)
        let cyS = Double(tileCy * tileScale)

        struct Solved {
            let tid: TileId
            let R: simd_float3x3
            let t: SIMD3<Float>
        }
        var solved: [Solved] = []

        let pnpTiles = tileCorr.values.filter {
            $0.pts3D.count >= config.pnpMinInliers
        }.count
        log("PnP on \(pnpTiles) of \(tileCorr.count) tiles")
        var pnpCalls = 0
        for (tid, data) in tileCorr {
            guard data.pts3D.count >= config.pnpMinInliers
            else { continue }
            pnpCalls += 1
            if pnpCalls <= 3 {
                log("PnP call \(pnpCalls), \(data.pts3D.count) points")
            }
            guard let res = RigCalibratorBridge.solvePnPRansac(
                data.pts3D, points2D: data.pts2D,
                fx: fS, fy: fS, cx: cxS, cy: cyS,
                minInliers: Int32(config.pnpMinInliers))
            else { continue }

            guard let Ra = res["R"] as? [NSNumber],
                  Ra.count >= 9,
                  let ta = res["tvec"] as? [NSNumber],
                  ta.count >= 3
            else { continue }
            solved.append(Solved(
                tid: tid,
                R: simd_float3x3(rows: [
                    SIMD3(Ra[0].floatValue, Ra[1].floatValue,
                          Ra[2].floatValue),
                    SIMD3(Ra[3].floatValue, Ra[4].floatValue,
                          Ra[5].floatValue),
                    SIMD3(Ra[6].floatValue, Ra[7].floatValue,
                          Ra[8].floatValue)]),
                t: SIMD3(ta[0].floatValue, ta[1].floatValue,
                         ta[2].floatValue)))
        }

        log("PnP solved \(solved.count)/\(facesTotal) tiles")
        guard solved.count >= config.minFaces else {
            return flag(
                "only \(solved.count) tiles solved"
                + " (need \(config.minFaces));"
                + " 360 images loaded \(x5Loaded)/\(calibX5.count),"
                + " 3D points \(pts3D.count)",
                fu: solved.count, ft: facesTotal)
        }

        // ── Phase 6: Fit rig correction ───────────────────
        log("phase 6: fit correction")

        let T_erp = COLMAPExporter.erpFromARKit(camCfg.extrinsic)
        let R_erp = ext3x3(T_erp)
        let t_erp = SIMD3<Float>(
            T_erp.columns.3.x, T_erp.columns.3.y,
            T_erp.columns.3.z)

        let rRotT = COLMAPExporter.portraitRotation.transpose

        var Qs: [simd_float3x3] = []
        var ds: [SIMD3<Float>] = []

        for s in solved {
            let scan = scans[s.tid.scanIndex]
            let fR = faceRots[s.tid.faceIndex]

            let x5Pose = interpX5(
                scan: scan, traj: trajectory,
                delay: config.x5ShutterDelay)

            // Sensor_from_rig rotation (matches writeRigsBin)
            let S_R = fR * R_erp
                * COLMAPExporter.arkitToColmap * rRotT
            let S_t = fR * t_erp

            // iPhone (rig ref) world-to-camera at X5 time
            let (R_l, t_mat) =
                COLMAPExporter.arkitToColmapW2C(x5Pose)
            let t_l = SIMD3<Float>(
                t_mat.columns.3.x, t_mat.columns.3.y,
                t_mat.columns.3.z)
            let fr_R = COLMAPExporter.portraitRotation * R_l
            let fr_t = COLMAPExporter.portraitRotation * t_l

            // Q = S^T * R_pnp * fr^T
            let Q = S_R.transpose * s.R * fr_R.transpose
            guard !hasNaN(Q) else { continue }

            // Centre correction in rig frame
            let c_old = -(S_R.transpose * S_t)
            let C_pnp = -(s.R.transpose * s.t)
            let C_rig = fr_R * C_pnp + fr_t
            let d = C_rig - c_old
            guard !d.x.isNaN, !d.y.isNaN,
                  !d.z.isNaN else { continue }
            ds.append(d)

            Qs.append(Q)
        }

        guard !Qs.isEmpty else {
            return flag("all tiles NaN",
                fu: solved.count, ft: facesTotal)
        }
        let (Q_avg, keep) = robustRotMean(Qs)
        let d_avg = medVec(ds.enumerated().compactMap {
            keep[$0.offset] ? $0.element : nil })

        let befDeg = Qs.map { angDeg($0) }
        let aftDeg = Qs.map { angDeg(Q_avg.transpose * $0) }
        let medBef = med(befDeg)
        let medAft = med(aftDeg)
        let corrDeg = angDeg(Q_avg)
        let corrCm = length(d_avg) * 100

        log(String(format:
            "correction %.2f° / %.1fcm; error %.2f° → %.2f°",
            corrDeg, corrCm, medBef, medAft))
        // d_avg is in the rig frame (iPhone camera: x right, y down,
        // z forward), so in Camera Settings Mount Position terms it is
        // forward = z, left = -x, up = -y. The export applies only the
        // rotation; adding these to the Mount Position moves the X5 to
        // where the images put it.
        log(String(format:
            "X5 position offset (not applied): forward %+.1f cm,"
                + " left %+.1f cm, up %+.1f cm",
            d_avg.z * 100, -d_avg.x * 100, -d_avg.y * 100))

        // ── Phase 7: Quality gate ─────────────────────────

        if corrDeg.isNaN || corrCm.isNaN
            || medBef.isNaN || medAft.isNaN
        {
            return flag(
                "NaN in calibration result",
                fu: solved.count, ft: facesTotal)
        }

        var fails: [String] = []
        if medAft > config.maxResidualDeg {
            fails.append(String(format:
                "residual %.2f° > %.1f°",
                medAft, config.maxResidualDeg))
        }
        if corrDeg > config.maxCorrectionDeg {
            fails.append(String(format:
                "correction %.1f° > %.0f°",
                corrDeg, config.maxCorrectionDeg))
        }
        if corrCm > config.maxCorrectionCm {
            fails.append(String(format:
                "shift %.1fcm > %.0fcm",
                corrCm, config.maxCorrectionCm))
        }
        if medAft >= medBef {
            fails.append("correction does not reduce error")
        }

        if !fails.isEmpty {
            return Result(
                status: .flagged,
                reason: fails.joined(separator: "; "),
                correctionAngleDeg: corrDeg,
                correctionShiftCm: corrCm,
                errorBeforeDeg: medBef,
                errorAfterDeg: medAft,
                facesUsed: solved.count,
                facesTotal: facesTotal,
                correctionQ: Q_avg,
                correctionD: d_avg,
                rigCorrectionForExport:
                    matrix_identity_float3x3)
        }

        // rigCorrection = pR^T * Q * pR
        let pR = COLMAPExporter.portraitRotation
        let rigCorr = pR.transpose * Q_avg * pR

        if hasNaN(rigCorr) {
            return flag(
                "NaN in rigCorrection matrix",
                fu: solved.count, ft: facesTotal)
        }

        return Result(
            status: .applied,
            reason: String(format:
                "%.2f° / %.1fcm; error %.2f° → %.2f°",
                corrDeg, corrCm, medBef, medAft),
            correctionAngleDeg: corrDeg,
            correctionShiftCm: corrCm,
            errorBeforeDeg: medBef,
            errorAfterDeg: medAft,
            facesUsed: solved.count,
            facesTotal: facesTotal,
            correctionQ: Q_avg,
            correctionD: d_avg,
            rigCorrectionForExport: rigCorr)
    }

    // MARK: - Projection matrices (portrait intrinsics)

    private static func buildPortraitProjMats(
        scans: [ScanExportData],
        entries: [IphoneEntry],
        maxWidth: Int
    ) -> [NSNumber: Data] {
        var result: [NSNumber: Data] = [:]
        for e in entries {
            let scan = scans[e.scanIndex]
            guard scan.intrinsics.count >= 2,
                  scan.intrinsics[0].count >= 3,
                  scan.intrinsics[1].count >= 3
            else { continue }

            // Landscape ARKit intrinsics
            let fxL = scan.intrinsics[0][0]
            let fyL = scan.intrinsics[1][1]
            let cxL = scan.intrinsics[0][2]
            let cyL = scan.intrinsics[1][2]
            let hL = Float(scan.imageHeight) // landscape H

            // Portrait intrinsics (90° CW from landscape)
            let fxP = fyL
            let fyP = fxL
            let cxP = hL - 1 - cyL
            let cyP = cxL

            // The portrait JPEG is hL×wL (width=hL, height=wL).
            // Its long side = wL = scan.imageWidth.
            let longSide = max(scan.imageWidth, scan.imageHeight)
            let scale: Float
            if maxWidth > 0, longSide > maxWidth {
                scale = Float(maxWidth) / Float(longSide)
            } else {
                scale = 1.0
            }

            let K = simd_float3x3(rows: [
                SIMD3(fxP * scale, 0, cxP * scale),
                SIMD3(0, fyP * scale, cyP * scale),
                SIMD3(0, 0, 1),
            ])

            let T_wc = scan.arkitPose
            let (R_l, t_mat) =
                COLMAPExporter.arkitToColmapW2C(T_wc)
            let t_l = SIMD3<Float>(
                t_mat.columns.3.x, t_mat.columns.3.y,
                t_mat.columns.3.z)
            let R = COLMAPExporter.portraitRotation * R_l
            let t = COLMAPExporter.portraitRotation * t_l

            let KR = K * R
            let Kt = K * t

            // 3×4 row-major doubles
            var P = [Double](repeating: 0, count: 12)
            for r in 0..<3 {
                P[r * 4 + 0] = Double(KR[0][r])
                P[r * 4 + 1] = Double(KR[1][r])
                P[r * 4 + 2] = Double(KR[2][r])
                P[r * 4 + 3] = Double(Kt[r])
            }
            result[NSNumber(value: e.imgId)] =
                Data(bytes: P, count: 12 * 8)
        }
        return result
    }

    // MARK: - Helpers

    private static func log(_ msg: String) {
        print("[RigCal] \(msg)")
        ExportTrace.note("rig calibration: \(msg)")
    }

    static func skip(_ reason: String) -> Result {
        log("skipped: \(reason)")
        return Result(
            status: .skipped, reason: reason,
            correctionAngleDeg: 0, correctionShiftCm: 0,
            errorBeforeDeg: 0, errorAfterDeg: 0,
            facesUsed: 0, facesTotal: 0,
            correctionQ: matrix_identity_float3x3,
            correctionD: .zero,
            rigCorrectionForExport: matrix_identity_float3x3)
    }

    private static func flag(
        _ reason: String, fu: Int, ft: Int
    ) -> Result {
        log("flagged: \(reason)")
        return Result(
            status: .flagged, reason: reason,
            correctionAngleDeg: 0, correctionShiftCm: 0,
            errorBeforeDeg: 0, errorAfterDeg: 0,
            facesUsed: fu, facesTotal: ft,
            correctionQ: matrix_identity_float3x3,
            correctionD: .zero,
            rigCorrectionForExport: matrix_identity_float3x3)
    }

    private static func obsKey(
        _ imgId: Int, _ x: Float, _ y: Float
    ) -> Int64 {
        Int64(imgId) * 100_000_000
            + Int64(round(x)) * 10_000
            + Int64(round(y))
    }

    private static func tileViewDir(
        scan: ScanExportData, face: Int,
        faceRots: [simd_float3x3],
        cam: CameraConfig,
        traj: [(timestamp: Double, pose: simd_float4x4)]?,
        delay: Double
    ) -> SIMD3<Float> {
        let x5 = interpX5(scan: scan, traj: traj, delay: delay)
        let T = COLMAPExporter.erpFromARKit(cam.extrinsic)
        let Re = ext3x3(T)
        let Ri = ext3x3(x5.inverse)
        let Rc = Re * COLMAPExporter.arkitToColmap
            * Ri * COLMAPExporter.arkitToColmap
        let Rt = faceRots[face] * Rc
        return normalize(Rt.transpose * SIMD3(0, 0, 1))
    }

    private static func interpX5(
        scan: ScanExportData,
        traj: [(timestamp: Double, pose: simd_float4x4)]?,
        delay: Double
    ) -> simd_float4x4 {
        if let ts = scan.insta360CommandTimestamp,
           let t = traj,
           let p = COLMAPExporter.interpolateTrajectory(
               t, at: ts + delay)
        { return p }
        return scan.arkitPose
    }

    private static func ext3x3(
        _ m: simd_float4x4
    ) -> simd_float3x3 {
        simd_float3x3(
            SIMD3(m.columns.0.x, m.columns.0.y, m.columns.0.z),
            SIMD3(m.columns.1.x, m.columns.1.y, m.columns.1.z),
            SIMD3(m.columns.2.x, m.columns.2.y, m.columns.2.z))
    }

    private static func rotToData(
        _ R: simd_float3x3
    ) -> Data {
        var buf = [Float](repeating: 0, count: 9)
        for r in 0..<3 {
            buf[r * 3 + 0] = R[0][r]
            buf[r * 3 + 1] = R[1][r]
            buf[r * 3 + 2] = R[2][r]
        }
        return Data(bytes: buf, count: 9 * 4)
    }

    private static func maskFilePath(
        cameraId: String, dir: URL?
    ) -> String? {
        guard let d = dir else { return nil }
        let p = d.appendingPathComponent("\(cameraId).png").path
        return FileManager.default.fileExists(atPath: p) ? p : nil
    }

    private static func rgbaToGray(
        _ rgba: [UInt8]
    ) -> [UInt8] {
        let n = rgba.count / 4
        var gray = [UInt8](repeating: 0, count: n)
        for i in 0..<n {
            let off = i * 4
            let r = UInt16(rgba[off])
            let g = UInt16(rgba[off + 1])
            let b = UInt16(rgba[off + 2])
            gray[i] = UInt8(
                (r * 77 + g * 150 + b * 29) >> 8)
        }
        return gray
    }

    private static func extractMaskTilesGray(
        maskPath: String?,
        slicer: ERPTileSlicer,
        nFaces: Int
    ) -> (tiles: [Data], w: Int32, h: Int32) {
        guard let mp = maskPath else { return ([], 0, 0) }
        let mBuf: (px: [UInt8], w: Int, h: Int)? =
            autoreleasepool {
                let url = URL(fileURLWithPath: mp)
                guard let source =
                    CGImageSourceCreateWithURL(
                        url as CFURL, nil),
                    let cg =
                        CGImageSourceCreateThumbnailAtIndex(
                            source, 0, [
                                kCGImageSourceThumbnailMaxPixelSize:
                                    2048,
                                kCGImageSourceCreateThumbnailFromImageAlways:
                                    true,
                                kCGImageSourceShouldCache:
                                    false,
                            ] as CFDictionary)
                else { return nil }
                let w = cg.width
                let h = cg.height
                guard w > 0, h > 0 else { return nil }
                var px = [UInt8](
                    repeating: 0, count: w * h * 4)
                guard let ctx = CGContext(
                    data: &px, width: w, height: h,
                    bitsPerComponent: 8,
                    bytesPerRow: w * 4,
                    space: CGColorSpaceCreateDeviceRGB(),
                    bitmapInfo: CGImageAlphaInfo
                        .noneSkipLast.rawValue)
                else { return nil }
                ctx.draw(cg, in: CGRect(
                    x: 0, y: 0, width: w, height: h))
                return (px, w, h)
            }
        guard let mBuf else { return ([], 0, 0) }

        var tiles: [Data] = []
        var tw: Int32 = 0, th: Int32 = 0
        for fi in 0..<nFaces {
            autoreleasepool {
                if let tile = slicer.sliceTile(
                    at: fi, srcPx: mBuf.px,
                    srcW: mBuf.w, srcH: mBuf.h),
                   let cg = tile.cgImage
                {
                    let mW = cg.width, mH = cg.height
                    var gm = [UInt8](
                        repeating: 0, count: mW * mH)
                    let cs =
                        CGColorSpaceCreateDeviceGray()
                    if let ctx = CGContext(
                        data: &gm, width: mW,
                        height: mH,
                        bitsPerComponent: 8,
                        bytesPerRow: mW, space: cs,
                        bitmapInfo: CGImageAlphaInfo
                            .none.rawValue)
                    {
                        ctx.translateBy(
                            x: 0, y: CGFloat(mH))
                        ctx.scaleBy(x: 1, y: -1)
                        ctx.draw(cg, in: CGRect(
                            x: 0, y: 0,
                            width: mW, height: mH))
                        tiles.append(Data(
                            bytes: gm,
                            count: mW * mH))
                        tw = Int32(mW)
                        th = Int32(mH)
                        return
                    }
                }
                tiles.append(Data())
            }
        }
        return (tiles, tw, th)
    }

    private static func hasNaN(
        _ m: simd_float3x3
    ) -> Bool {
        for c in 0..<3 {
            if m[c].x.isNaN || m[c].y.isNaN
                || m[c].z.isNaN { return true }
        }
        return false
    }

    private static func angDeg(_ R: simd_float3x3) -> Float {
        let tr = R[0][0] + R[1][1] + R[2][2]
        return acos(min(max((tr - 1) / 2, -1), 1)) * 180 / .pi
    }

    private static func med(_ a: [Float]) -> Float {
        guard !a.isEmpty else { return 0 }
        let s = a.sorted()
        let m = s.count / 2
        return s.count % 2 == 0
            ? (s[m - 1] + s[m]) / 2 : s[m]
    }

    private static func medVec(
        _ v: [SIMD3<Float>]
    ) -> SIMD3<Float> {
        guard !v.isEmpty else { return .zero }
        return SIMD3(med(v.map(\.x)),
                     med(v.map(\.y)),
                     med(v.map(\.z)))
    }

    private static func robustRotMean(
        _ rots: [simd_float3x3], iter: Int = 3
    ) -> (simd_float3x3, [Bool]) {
        var keep = [Bool](repeating: true, count: rots.count)
        var mean = rots[0]
        for _ in 0..<iter {
            let kept = rots.enumerated().compactMap {
                keep[$0.offset] ? $0.element : nil }
            guard !kept.isEmpty else { break }
            mean = avgRot(kept)
            let err = rots.map { angDeg(mean.transpose * $0) }
            let keptErr = err.enumerated().compactMap {
                keep[$0.offset] ? $0.element : nil }
            let thr = max(Float(1), 3 * med(keptErr))
            let newKeep = err.map { $0 <= thr }
            if newKeep.filter(\.self).count < 3
                || newKeep == keep { break }
            keep = newKeep
        }
        return (mean, keep)
    }

    private static func avgRot(
        _ rots: [simd_float3x3]
    ) -> simd_float3x3 {
        guard !rots.isEmpty else {
            return matrix_identity_float3x3 }
        var acc = simd_quatf(rots[0])
        for i in 1..<rots.count {
            var q = simd_quatf(rots[i])
            if simd_dot(acc, q) < 0 { q = -q }
            acc = simd_quatf(
                ix: acc.imag.x + q.imag.x,
                iy: acc.imag.y + q.imag.y,
                iz: acc.imag.z + q.imag.z,
                r: acc.real + q.real)
        }
        let n = simd_length(simd_float4(
            acc.imag.x, acc.imag.y, acc.imag.z, acc.real))
        acc = simd_quatf(
            ix: acc.imag.x / n, iy: acc.imag.y / n,
            iz: acc.imag.z / n, r: acc.real / n)
        return simd_float3x3(acc)
    }
}

// MARK: - Union-Find

private struct UnionFind {
    private var parent: [Int64: Int64] = [:]

    mutating func find(_ x: Int64) -> Int64 {
        if parent[x] == nil { parent[x] = x }
        var cur = x
        while parent[cur] != cur {
            parent[cur] = parent[parent[cur]!]
            cur = parent[cur]!
        }
        return cur
    }

    mutating func unite(_ a: Int64, _ b: Int64) {
        let ra = find(a), rb = find(b)
        if ra != rb { parent[rb] = ra }
    }
}
