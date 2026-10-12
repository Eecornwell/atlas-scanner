import Foundation
import simd
import UIKit

/// Nelder-Mead 6-DOF optimiser for T_insta_iphone.
///
/// Cost function: for each matched (lidar_pixel, insta_pixel) pair,
/// convert the LiDAR pixel to a bearing vector in iPhone camera frame,
/// project it through the candidate T_insta_iphone into ERP coordinates,
/// and measure the squared pixel distance to the matched Insta360 keypoint.
///
/// This is the same reprojection cost as calibrate_extrinsic.py but driven
/// by real KAZE+FLANN matched keypoints (via FeatureMatcher) instead of
/// dense bearing vectors.
enum ExtrinsicOptimizer {

    /// Rz(-90°) in ERP space: corrects for the 90° mismatch between
    /// the landscape depth buffer's bearing frame and the ERP convention.
    /// Empirically confirmed: physical seed (0,-10,-90) produces correct
    /// overlay alignment with this correction applied.
    static let erpCorrection = simd_float3x3(rows: [
        SIMD3<Float>( 0,  1,  0),
        SIMD3<Float>(-1,  0,  0),
        SIMD3<Float>( 0,  0,  1),
    ])

    // MARK: - Public

    /// Matched pair: LiDAR bearing in iPhone camera frame + Insta360 ERP pixel.
    struct MatchSample {
        let lidarBearing: SIMD3<Float>  // unit vector in iPhone camera frame
        let instaU: Float               // matched pixel in Insta360 ERP (0..erpW)
        let instaV: Float               // matched pixel in Insta360 ERP (0..erpH)
    }

    /// Build match samples from FeatureMatcher output + depth map.
    /// Converts each matched LiDAR ERP pixel back to a 3D bearing via the
    /// depth map and iPhone intrinsics.
    static func buildSamples(
        pairs: [MatchedPair],
        frame: CalibrationFrame,
        lidarERPSize: CGSize,       // size of the LiDAR intensity ERP image
        instaERPSize: CGSize
    ) -> [MatchSample] {
        let lidarERPW = Float(lidarERPSize.width)
        let lidarERPH = Float(lidarERPSize.height)

        let di = frame.depthIntrinsics()
        let fx = di.fx, fy = di.fy, cx = di.cx, cy = di.cy

        var samples: [MatchSample] = []
        samples.reserveCapacity(pairs.count)

        for pair in pairs {
            // 1. LiDAR ERP pixel → spherical bearing in Insta360 frame
            let lu = Float(pair.lidarPt.x) / lidarERPW
            let lv = Float(pair.lidarPt.y) / lidarERPH
            let lon = (lu - 0.5) * 2 * Float.pi
            let lat = -(lv - 0.5) * Float.pi   // lat = -asin(Y) → Y = -sin(lat)

            // Bearing in Insta360 frame (ERP convention: X=atan2, Z=forward)
            let bx = sin(lon) * cos(lat)
            let by = -sin(lat)
            let bz = cos(lon) * cos(lat)
            let bInsta = SIMD3<Float>(bx, by, bz)

            // 2. Find the depth value closest to this bearing direction
            //    by projecting the bearing back through the seed T to iPhone frame
            //    and looking up the nearest depth pixel.
            //    We use the bearing directly as the sample — the optimizer will
            //    find T such that T * lidarBearing ≈ bInsta.
            //    lidarBearing is bInsta transformed by inv(T_seed) — but since
            //    we're optimizing T, we store bInsta as the "target" and derive
            //    the source bearing from the depth map lookup below.

            // Find depth pixel nearest to this ERP direction
            // Project bInsta through seed T_inv to get approximate iPhone camera ray
            // then look up depth at that pixel
            let depthCol = Int((lon / (2 * .pi) + 0.5) * Float(frame.depthW))
            let depthRow = Int((0.5 - lat / .pi) * Float(frame.depthH))
            let dc = max(0, min(frame.depthW - 1, depthCol))
            let dr = max(0, min(frame.depthH - 1, depthRow))
            let z = frame.depth[dr * frame.depthW + dc]
            guard z > 0.1 && z < 5.0 else { continue }

            // Back-project depth pixel to Camera frame (X=up, Y=right, Z=forward).
            // Portrait-rotated intrinsics produce (down, left, forward); negate XY.
            let xCam = (Float(dc) - cx) / fx * z
            let yCam = (Float(dr) - cy) / fy * z
            let pt = SIMD3<Float>(-xCam, -yCam, z)
            let norm = simd_length(pt)
            guard norm > 0 else { continue }

            samples.append(MatchSample(
                lidarBearing: pt / norm,
                instaU: Float(pair.instaPt.x),
                instaV: Float(pair.instaPt.y)
            ))
        }
        return samples
    }

    /// Run Nelder-Mead optimisation given pre-built match samples.
    static func optimize(
        samples: [MatchSample],
        seed: RigidTransform,
        erpW: Int,
        erpH: Int,
        maxIterations: Int = 400
    ) -> (result: RigidTransform, finalCost: Float) {
        guard !samples.isEmpty else { return (seed, Float.infinity) }

        let nm = nelderMead(
            start: seedToParams(seed),
            cost: { p in reprojectionCost(p, samples: samples, erpW: erpW, erpH: erpH) },
            maxIter: maxIterations
        )
        return (paramsToRigidTransform(nm.params), nm.cost)
    }

    // MARK: - Reprojection cost

    /// Mean squared ERP pixel distance between projected LiDAR bearing and
    /// matched Insta360 keypoint. Mirrors calibrate_extrinsic.py reprojection_cost().
    private static func reprojectionCost(
        _ params: [Float],
        samples: [MatchSample],
        erpW: Int,
        erpH: Int
    ) -> Float {
        let T = paramsToMatrix(params)
        let R = simd_float3x3(
            SIMD3(T.columns.0.x, T.columns.0.y, T.columns.0.z),
            SIMD3(T.columns.1.x, T.columns.1.y, T.columns.1.z),
            SIMD3(T.columns.2.x, T.columns.2.y, T.columns.2.z)
        )
        let W = Float(erpW), H = Float(erpH)
        var total: Float = 0

        for s in samples {
            let b = erpCorrection * simd_normalize(R * s.lidarBearing)
            let lat = -asin(max(-1, min(1, b.y)))
            let lon = atan2(b.x, b.z)
            var u = W * (0.5 + lon / (2 * .pi))
            let v = H * (0.5 - lat / .pi)

            // Wrap horizontal distance
            var du = u - s.instaU
            if abs(du) > W / 2 { du -= du.sign * W }
            let dv = v - s.instaV
            total += du * du + dv * dv
        }
        return total / Float(samples.count)
    }

    // MARK: - Nelder-Mead

    private struct NMResult { let params: [Float]; let cost: Float }

    private static func nelderMead(
        start: [Float],
        steps: [Float]? = nil,
        cost: ([Float]) -> Float,
        maxIter: Int
    ) -> NMResult {
        let n = start.count
        let alpha: Float = 1.0, gamma: Float = 2.0, rho: Float = 0.5, sigma: Float = 0.5
        var simplex = [[Float]](repeating: start, count: n + 1)
        let s = steps ?? [0.035, 0.035, 0.035, 0.005, 0.005, 0.005]
        for i in 0..<n { simplex[i + 1][i] += s[min(i, s.count - 1)] }
        var costs = simplex.map { cost($0) }

        for _ in 0..<maxIter {
            let order = costs.indices.sorted { costs[$0] < costs[$1] }
            simplex = order.map { simplex[$0] }
            costs   = order.map { costs[$0] }
            if costs[n] - costs[0] < 1e-8 { break }

            var centroid = [Float](repeating: 0, count: n)
            for i in 0..<n { for j in 0..<n { centroid[j] += simplex[i][j] / Float(n) } }

            let reflected = zip(centroid, simplex[n]).map { c, w in c + alpha * (c - w) }
            let cr = cost(reflected)

            if cr < costs[0] {
                let expanded = zip(centroid, reflected).map { c, r in c + gamma * (r - c) }
                let ce = cost(expanded)
                simplex[n] = ce < cr ? expanded : reflected
                costs[n]   = ce < cr ? ce : cr
            } else if cr < costs[n - 1] {
                simplex[n] = reflected; costs[n] = cr
            } else {
                let contracted = zip(centroid, simplex[n]).map { c, w in c + rho * (w - c) }
                let cc = cost(contracted)
                if cc < costs[n] {
                    simplex[n] = contracted; costs[n] = cc
                } else {
                    for i in 1...n {
                        simplex[i] = zip(simplex[0], simplex[i]).map { b, s in b + sigma * (s - b) }
                        costs[i] = cost(simplex[i])
                    }
                }
            }
        }
        return NMResult(params: simplex[0], cost: costs[0])
    }

    // MARK: - Edge-based optimization (no OpenCV)

    struct LiDARBearing {
        let bearing: SIMD3<Float>
    }

    static func buildBearings(frame: CalibrationFrame) -> [LiDARBearing] {
        let di = frame.depthIntrinsics()
        let fx = di.fx, fy = di.fy, cx = di.cx, cy = di.cy
        let w = frame.depthW, h = frame.depthH

        var depthGrad = [Float](repeating: 0, count: w * h)
        for row in 1..<(h - 1) {
            for col in 1..<(w - 1) {
                let c = frame.depth[row * w + col]
                guard c > 0.1 else { continue }
                let l = frame.depth[row * w + col - 1]
                let r = frame.depth[row * w + col + 1]
                let u = frame.depth[(row - 1) * w + col]
                let d = frame.depth[(row + 1) * w + col]
                let gx = abs(r - l) / c
                let gy = abs(d - u) / c
                depthGrad[row * w + col] = max(gx, gy)
            }
        }

        var grads: [Float] = []
        for row in 1..<(h - 1) {
            for col in 1..<(w - 1) {
                let g = depthGrad[row * w + col]
                if g > 0.01 { grads.append(g) }
            }
        }
        grads.sort()
        let threshold: Float
        if grads.count > 100 {
            threshold = grads[grads.count - min(grads.count, 2000)]
        } else {
            threshold = 0.05
        }

        var bearings: [LiDARBearing] = []
        bearings.reserveCapacity(2000)

        for row in 1..<(h - 1) {
            for col in 1..<(w - 1) {
                guard depthGrad[row * w + col] >= threshold else {
                    continue
                }
                let z = frame.depth[row * w + col]
                guard z > 0.15 && z < 5.0 else { continue }
                let xCam = (Float(col) - cx) / fx * z
                let yCam = (Float(row) - cy) / fy * z
                let pt = SIMD3<Float>(-xCam, -yCam, z)
                let norm = simd_length(pt)
                guard norm > 0.05 else { continue }
                bearings.append(LiDARBearing(bearing: pt / norm))
            }
        }
        return bearings
    }

    static func sobelEdges(from image: UIImage) -> (edges: [Float], w: Int, h: Int)? {
        guard let cg = image.cgImage else { return nil }
        let w = cg.width, h = cg.height
        var gray = [UInt8](repeating: 0, count: w * h)
        guard let ctx = CGContext(
            data: &gray, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w,
            space: CGColorSpaceCreateDeviceGray(),
            bitmapInfo: CGImageAlphaInfo.none.rawValue
        ) else { return nil }
        ctx.draw(cg, in: CGRect(x: 0, y: 0, width: w, height: h))

        var out = [Float](repeating: 0, count: w * h)
        for y in 1..<(h - 1) {
            for x in 1..<(w - 1) {
                let gx = -Int(gray[(y-1)*w+(x-1)]) - 2*Int(gray[y*w+(x-1)])
                    - Int(gray[(y+1)*w+(x-1)])
                    + Int(gray[(y-1)*w+(x+1)]) + 2*Int(gray[y*w+(x+1)])
                    + Int(gray[(y+1)*w+(x+1)])
                let gy = -Int(gray[(y-1)*w+(x-1)]) - 2*Int(gray[(y-1)*w+x])
                    - Int(gray[(y-1)*w+(x+1)])
                    + Int(gray[(y+1)*w+(x-1)]) + 2*Int(gray[(y+1)*w+x])
                    + Int(gray[(y+1)*w+(x+1)])
                out[y * w + x] = Float(gx * gx + gy * gy)
            }
        }
        let maxV = out.max() ?? 1
        if maxV > 0 { for i in 0..<out.count { out[i] /= maxV } }
        return (out, w, h)
    }

    static func optimizeEdgeBased(
        bearings: [LiDARBearing],
        edgeMap: [Float],
        edgeW: Int,
        edgeH: Int,
        seed: RigidTransform,
        mask: [Bool]? = nil,
        maxIterations: Int = 1500
    ) -> (result: RigidTransform, finalCost: Float) {
        guard !bearings.isEmpty else { return (seed, Float.infinity) }

        let nm = nelderMead(
            start: seedToParams(seed),
            cost: { p in
                edgeCost(
                    p, bearings: bearings,
                    edgeMap: edgeMap, edgeW: edgeW, edgeH: edgeH,
                    mask: mask
                )
            },
            maxIter: maxIterations
        )
        return (paramsToRigidTransform(nm.params), nm.cost)
    }

    /// Loads an ERP mask image and returns a Bool array (true = masked).
    static func loadMaskBool(
        _ maskImage: UIImage?, w: Int, h: Int
    ) -> [Bool]? {
        guard let cg = maskImage?.cgImage else { return nil }
        var gray = [UInt8](repeating: 0, count: w * h)
        guard let ctx = CGContext(
            data: &gray, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w,
            space: CGColorSpaceCreateDeviceGray(),
            bitmapInfo: CGImageAlphaInfo.none.rawValue
        ) else { return nil }
        ctx.draw(cg, in: CGRect(x: 0, y: 0, width: w, height: h))
        return gray.map { $0 > 128 }
    }

    private static func edgeCost(
        _ params: [Float],
        bearings: [LiDARBearing],
        edgeMap: [Float],
        edgeW: Int,
        edgeH: Int,
        mask: [Bool]?
    ) -> Float {
        let T = paramsToMatrix(params)
        let R = simd_float3x3(
            SIMD3(T.columns.0.x, T.columns.0.y, T.columns.0.z),
            SIMD3(T.columns.1.x, T.columns.1.y, T.columns.1.z),
            SIMD3(T.columns.2.x, T.columns.2.y, T.columns.2.z)
        )
        let W = Float(edgeW), H = Float(edgeH)
        var total: Float = 0
        var count = 0

        for b in bearings {
            let rotated = erpCorrection * simd_normalize(R * b.bearing)
            let lat = -asin(max(-1, min(1, rotated.y)))
            let lon = atan2(rotated.x, rotated.z)
            let u = Int(W * (0.5 + lon / (2 * .pi))) % edgeW
            let v = max(0, min(edgeH - 1, Int(H * (0.5 - lat / .pi))))
            guard u >= 0 else { continue }
            let idx = v * edgeW + u
            if let m = mask, m[idx] { continue }
            total += edgeMap[idx]
            count += 1
        }
        guard count > 0 else { return 0 }
        return -total / Float(count)
    }

    // MARK: - Photometric optimization (dense NCC)

    static func optimizePhotometric(
        iphoneGray: [UInt8],
        outW: Int,
        outH: Int,
        erpGray: [UInt8],
        erpW: Int,
        erpH: Int,
        fx: Float, fy: Float, cx: Float, cy: Float,
        seed: RigidTransform,
        maxIterations: Int = 800
    ) -> (result: RigidTransform, finalCost: Float) {
        let n = outW * outH
        guard n > 0 else { return (seed, 0) }

        // Neutralize Insta360 stitch seam — replace with median gray
        // so seam artifacts don't create false NCC signal.
        let seam = seamColumns(erpW: erpW)
        var cleanErp = erpGray
        if !seam.isEmpty {
            let sorted = erpGray.sorted()
            let median = sorted[erpGray.count / 2]
            for col in seam {
                for row in 0..<erpH {
                    cleanErp[row * erpW + col] = median
                }
            }
        }

        // Precompute iPhone statistics for NCC
        var sumA: Float = 0
        var sumA2: Float = 0
        for v in iphoneGray {
            let fv = Float(v)
            sumA += fv
            sumA2 += fv * fv
        }
        let meanA = sumA / Float(n)
        let stdA = sqrt(max(0, sumA2 - Float(n) * meanA * meanA))
        guard stdA > 1 else { return (seed, 0) }

        // Precompute ray directions (camera frame, don't change per eval)
        var rays = [SIMD3<Float>](repeating: .zero, count: n)
        for row in 0..<outH {
            for col in 0..<outW {
                let xCam = (Float(col) - cx) / fx
                let yCam = (Float(row) - cy) / fy
                rays[row * outW + col] = simd_normalize(
                    SIMD3<Float>(-xCam, -yCam, 1.0)
                )
            }
        }

        let eWf = Float(erpW), eHf = Float(erpH)

        let startParams: [Float] = [
            Float(seed.roll * .pi / 180),
            Float(seed.pitch * .pi / 180),
            Float(seed.yaw * .pi / 180),
        ]

        let nm = nelderMead(
            start: startParams,
            steps: [0.087, 0.087, 0.087],
            cost: { params in
                let r = params[0], p = params[1], y = params[2]
                let cr = cos(r), sr = sin(r)
                let cp = cos(p), sp = sin(p)
                let cy = cos(y), sy = sin(y)
                let rSeed = simd_float3x3(rows: [
                    SIMD3(cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr),
                    SIMD3(sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
                    SIMD3(-sp,   cp*sr,            cp*cr),
                ])
                let R = basisC * rSeed * basisD

                var sumB: Float = 0
                var sumB2: Float = 0
                var sumAB: Float = 0

                for i in 0..<n {
                    let rotated = erpCorrection * (R * rays[i])
                    let lat = -asin(max(-1, min(1, rotated.y)))
                    let lon = atan2(rotated.x, rotated.z)

                    var uf = eWf * (0.5 + lon / (2 * .pi)) - 0.5
                    let vf = eHf * (0.5 - lat / .pi) - 0.5
                    if uf < 0 { uf += eWf }
                    if uf >= eWf { uf -= eWf }

                    let u0 = Int(uf)
                    let v0 = max(0, min(erpH - 1, Int(vf)))
                    let u1 = (u0 + 1) % erpW
                    let v1 = min(erpH - 1, v0 + 1)
                    let fu = uf - Float(u0)
                    let fv = vf - Float(v0)

                    let p00 = Float(cleanErp[v0 * erpW + u0])
                    let p10 = Float(cleanErp[v0 * erpW + u1])
                    let p01 = Float(cleanErp[v1 * erpW + u0])
                    let p11 = Float(cleanErp[v1 * erpW + u1])
                    let val = (p00 + (p10 - p00) * fu)
                        * (1 - fv)
                        + (p01 + (p11 - p01) * fu) * fv

                    sumB += val
                    sumB2 += val * val
                    sumAB += Float(iphoneGray[i]) * val
                }

                let meanB = sumB / Float(n)
                let stdB = sqrt(
                    max(0, sumB2 - Float(n) * meanB * meanB)
                )
                guard stdB > 1 else { return Float(0) }
                let ncc = (sumAB - Float(n) * meanA * meanB)
                    / (stdA * stdB)
                return -ncc
            },
            maxIter: maxIterations
        )

        let result = RigidTransform(
            roll: Double(nm.params[0] * 180 / .pi),
            pitch: Double(nm.params[1] * 180 / .pi),
            yaw: Double(nm.params[2] * 180 / .pi),
            x: seed.x, y: seed.y, z: seed.z
        )
        return (result, nm.cost)
    }

    // MARK: - Stitch seam masking

    /// Returns ERP column indices that fall within the Insta360 stitch seam.
    /// The two fisheye lenses meet at lon=±90° → columns W/4 and 3W/4.
    /// Band width scales with resolution (~2.5° each side).
    static func seamColumns(erpW: Int) -> Set<Int> {
        let band = max(15, erpW / 100)
        let centers = [erpW / 4, erpW * 3 / 4]
        var cols = Set<Int>()
        for c in centers {
            for dx in -band...band {
                cols.insert(((c + dx) % erpW + erpW) % erpW)
            }
        }
        return cols
    }

    // MARK: - Coordinate frame conversion
    //
    // Three frames (all right-handed):
    //   Seed:   X=forward, Y=left, Z=up     (physical mount measurements)
    //   Camera: X=up, Y=right, Z=forward    (portrait iPhone intrinsics)
    //   ERP:    X=right, Y=down, Z=forward  (Insta360 pinhole / ERP projection)
    //
    // Full rotation:    R = basisC * R_seed * basisD
    // Full translation: t = basisC * t_seed
    //
    // basisC maps seed-frame → ERP-frame directions.
    // basisD maps camera-frame → seed-frame directions.

    private static let basisC = simd_float3x3(rows: [
        SIMD3<Float>( 0, -1,  0),
        SIMD3<Float>( 0,  0, -1),
        SIMD3<Float>( 1,  0,  0),
    ])

    private static let basisD = simd_float3x3(rows: [
        SIMD3<Float>( 0,  0,  1),
        SIMD3<Float>( 0, -1,  0),
        SIMD3<Float>( 1,  0,  0),
    ])

    static func seedToParams(_ t: RigidTransform) -> [Float] {
        [Float(t.roll * .pi / 180),
         Float(t.pitch * .pi / 180),
         Float(t.yaw * .pi / 180),
         Float(t.x), Float(t.y), Float(t.z)]
    }

    static func paramsToMatrix(_ p: [Float]) -> simd_float4x4 {
        seedToCameraMatrix(paramsToRigidTransform(p))
    }

    static func paramsToRigidTransform(_ p: [Float]) -> RigidTransform {
        RigidTransform(
            roll:  Double(p[0] * 180 / .pi),
            pitch: Double(p[1] * 180 / .pi),
            yaw:   Double(p[2] * 180 / .pi),
            x: Double(p[3]), y: Double(p[4]), z: Double(p[5])
        )
    }

    static func seedToCameraMatrix(_ t: RigidTransform) -> simd_float4x4 {
        let r = Float(t.roll * .pi / 180)
        let p = Float(t.pitch * .pi / 180)
        let y = Float(t.yaw * .pi / 180)
        let cr = cos(r), sr = sin(r)
        let cp = cos(p), sp = sin(p)
        let cy = cos(y), sy = sin(y)

        let rSeed = simd_float3x3(rows: [
            SIMD3(cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr),
            SIMD3(sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
            SIMD3(-sp,   cp*sr,            cp*cr),
        ])

        let rot = basisC * rSeed * basisD
        let tSeed = SIMD3<Float>(Float(t.x), Float(t.y), Float(t.z))
        let tErp = basisC * tSeed

        var M = simd_float4x4(1)
        M.columns.0 = SIMD4(rot.columns.0, 0)
        M.columns.1 = SIMD4(rot.columns.1, 0)
        M.columns.2 = SIMD4(rot.columns.2, 0)
        M.columns.3 = SIMD4(tErp, 1)
        return M
    }
}

private extension FloatingPoint {
    var sign: Self { self >= 0 ? 1 : -1 }
}
