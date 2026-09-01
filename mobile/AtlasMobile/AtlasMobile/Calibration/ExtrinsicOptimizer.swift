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

        // Scale intrinsics to LiDAR depth resolution
        let scaleX = Float(frame.depthW) / Float(frame.imageW)
        let scaleY = Float(frame.depthH) / Float(frame.imageH)
        let fx = frame.intrinsics[0][0] * scaleX
        let fy = frame.intrinsics[1][1] * scaleY
        let cx = frame.intrinsics[2][0] * scaleX
        let cy = frame.intrinsics[2][1] * scaleY

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

            // Back-project depth pixel to iPhone camera frame bearing
            let xCam = (Float(dc) - cx) / fx * z
            let yCam = (Float(dr) - cy) / fy * z
            let pt = SIMD3<Float>(xCam, yCam, z)
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
            let b = simd_normalize(R * s.lidarBearing)
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

    // MARK: - Nelder-Mead (unchanged)

    private struct NMResult { let params: [Float]; let cost: Float }

    private static func nelderMead(
        start: [Float],
        cost: ([Float]) -> Float,
        maxIter: Int
    ) -> NMResult {
        let n = start.count
        let alpha: Float = 1.0, gamma: Float = 2.0, rho: Float = 0.5, sigma: Float = 0.5
        var simplex = [[Float]](repeating: start, count: n + 1)
        let steps: [Float] = [0.05, 0.05, 0.05, 0.005, 0.005, 0.005]
        for i in 0..<n { simplex[i + 1][i] += steps[i] }
        var costs = simplex.map { cost($0) }

        for _ in 0..<maxIter {
            let order = costs.indices.sorted { costs[$0] < costs[$1] }
            simplex = order.map { simplex[$0] }
            costs   = order.map { costs[$0] }
            if costs[n] - costs[0] < 1e-6 { break }

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

    // MARK: - Param helpers

    static func seedToParams(_ t: RigidTransform) -> [Float] {
        [Float(t.roll * .pi / 180), Float(t.pitch * .pi / 180), Float(t.yaw * .pi / 180),
         Float(t.x), Float(t.y), Float(t.z)]
    }

    static func paramsToMatrix(_ p: [Float]) -> simd_float4x4 {
        let r = p[0], pitch = p[1], y = p[2]
        let cr = cos(r), sr = sin(r), cp = cos(pitch), sp = sin(pitch), cy = cos(y), sy = sin(y)
        let rot = simd_float3x3(rows: [
            SIMD3(cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr),
            SIMD3(sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
            SIMD3(-sp,   cp*sr,            cp*cr)
        ])
        var T = simd_float4x4(1)
        T.columns.0 = SIMD4(rot.columns.0, 0)
        T.columns.1 = SIMD4(rot.columns.1, 0)
        T.columns.2 = SIMD4(rot.columns.2, 0)
        T.columns.3 = SIMD4(p[3], p[4], p[5], 1)
        return T
    }

    static func paramsToRigidTransform(_ p: [Float]) -> RigidTransform {
        RigidTransform(
            roll:  Double(p[0] * 180 / .pi),
            pitch: Double(p[1] * 180 / .pi),
            yaw:   Double(p[2] * 180 / .pi),
            x: Double(p[3]), y: Double(p[4]), z: Double(p[5])
        )
    }
}

private extension FloatingPoint {
    var sign: Self { self >= 0 ? 1 : -1 }
}
