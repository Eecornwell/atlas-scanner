import Foundation
import UIKit

// MARK: - Data Types

struct Keypoint {
    let x: Int
    let y: Int
    let score: Float
    let angle: Float
}

/// 256-bit binary descriptor (ORB / rotated BRIEF).
struct BinaryDescriptor {
    let d0: UInt64
    let d1: UInt64
    let d2: UInt64
    let d3: UInt64
}

struct FeatureMatch {
    let queryIdx: Int
    let trainIdx: Int
    let distance: Int
}

// MARK: - FAST-9 + ORB Feature Detector
//
// Commercial-friendly — no patented algorithms:
//   FAST: Rosten & Drummond (BSD), corner detection on Bresenham circle
//   BRIEF: Calonder et al. (EPFL, no patent), binary pixel-pair descriptor
//   ORB: Rublee et al. (no patent), adds orientation to BRIEF
//
// Pipeline:
//   1. 3×3 box blur for noise reduction
//   2. FAST-9 corner detection with adaptive threshold
//   3. Grid-based non-maximum suppression
//   4. Intensity-centroid orientation (ORB)
//   5. Rotated BRIEF 256-bit descriptor
//   6. Brute-force Hamming matching with Lowe's ratio test

enum FeatureDetector {

    // MARK: - Public API

    static func detectAndDescribe(
        image: UIImage,
        maxKeypoints: Int = 1500
    ) -> (keypoints: [Keypoint], descriptors: [BinaryDescriptor])? {
        guard let (gray, w, h) = toGrayscale(image) else {
            return nil
        }
        return detectAndDescribe(
            gray: gray, width: w, height: h,
            maxKeypoints: maxKeypoints
        )
    }

    static func detectAndDescribe(
        gray: [UInt8], width w: Int, height h: Int,
        maxKeypoints: Int = 1500,
        fastThreshold: Int = 20
    ) -> (keypoints: [Keypoint], descriptors: [BinaryDescriptor]) {
        let blurred = boxBlur3x3(gray, w: w, h: h)

        var kps = detectFAST9(blurred, w: w, h: h, threshold: fastThreshold)
        kps = nonMaxSuppression(kps, w: w, h: h, cellSize: 8)
        kps.sort { $0.score > $1.score }
        if kps.count > maxKeypoints {
            kps = Array(kps.prefix(maxKeypoints))
        }

        let margin = patchRadius
        kps = kps.filter {
            $0.x >= margin && $0.x < w - margin &&
            $0.y >= margin && $0.y < h - margin
        }

        kps = kps.map { kp in
            let angle = computeOrientation(
                blurred, w: w, h: h, kp: kp
            )
            return Keypoint(
                x: kp.x, y: kp.y,
                score: kp.score, angle: angle
            )
        }

        let descs = kps.map { kp in
            computeDescriptor(blurred, w: w, h: h, kp: kp)
        }

        return (kps, descs)
    }

    /// Brute-force Hamming matching with Lowe's ratio test.
    static func match(
        query: [BinaryDescriptor],
        train: [BinaryDescriptor],
        maxDistance: Int = 64,
        ratioThreshold: Float = 0.75
    ) -> [FeatureMatch] {
        guard !query.isEmpty, !train.isEmpty else { return [] }
        var matches: [FeatureMatch] = []
        matches.reserveCapacity(min(query.count, train.count))

        for (qi, qd) in query.enumerated() {
            var best = Int.max
            var secondBest = Int.max
            var bestIdx = -1

            for (ti, td) in train.enumerated() {
                let d = hammingDistance(qd, td)
                if d < best {
                    secondBest = best
                    best = d
                    bestIdx = ti
                } else if d < secondBest {
                    secondBest = d
                }
            }

            guard best <= maxDistance, bestIdx >= 0 else { continue }
            guard secondBest > 0 else { continue }

            if Float(best) / Float(secondBest) < ratioThreshold {
                matches.append(FeatureMatch(
                    queryIdx: qi, trainIdx: bestIdx,
                    distance: best
                ))
            }
        }

        return matches
    }

    /// Cross-checked matching — both directions must agree.
    static func matchCrossCheck(
        desc1: [BinaryDescriptor],
        desc2: [BinaryDescriptor],
        maxDistance: Int = 64
    ) -> [FeatureMatch] {
        guard !desc1.isEmpty, !desc2.isEmpty else { return [] }

        let forward = bestMatches(from: desc1, to: desc2)
        let backward = bestMatches(from: desc2, to: desc1)

        var matches: [FeatureMatch] = []
        for (qi, ti) in forward.enumerated() {
            guard ti >= 0,
                  forward[qi] >= 0,
                  backward[ti] == qi else { continue }
            let d = hammingDistance(desc1[qi], desc2[ti])
            guard d <= maxDistance else { continue }
            matches.append(FeatureMatch(
                queryIdx: qi, trainIdx: ti, distance: d
            ))
        }
        return matches
    }

    // MARK: - Grayscale Conversion

    static func toGrayscale(
        _ image: UIImage
    ) -> (pixels: [UInt8], width: Int, height: Int)? {
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
        return (gray, w, h)
    }

    // MARK: - FAST-9 Corner Detection

    private static let circleOffsets: [(dx: Int, dy: Int)] = [
        ( 0, -3), ( 1, -3), ( 2, -2), ( 3, -1),
        ( 3,  0), ( 3,  1), ( 2,  2), ( 1,  3),
        ( 0,  3), (-1,  3), (-2,  2), (-3,  1),
        (-3,  0), (-3, -1), (-2, -2), (-1, -3),
    ]

    private static func detectFAST9(
        _ gray: [UInt8], w: Int, h: Int, threshold: Int
    ) -> [Keypoint] {
        var keypoints: [Keypoint] = []
        keypoints.reserveCapacity(w * h / 64)

        for y in 3..<(h - 3) {
            for x in 3..<(w - 3) {
                let center = Int(gray[y * w + x])
                let tHi = center + threshold
                let tLo = center - threshold

                let p0  = Int(gray[(y - 3) * w + x])
                let p4  = Int(gray[y * w + (x + 3)])
                let p8  = Int(gray[(y + 3) * w + x])
                let p12 = Int(gray[y * w + (x - 3)])

                let nB = (p0 > tHi ? 1 : 0) + (p4 > tHi ? 1 : 0)
                       + (p8 > tHi ? 1 : 0) + (p12 > tHi ? 1 : 0)
                let nD = (p0 < tLo ? 1 : 0) + (p4 < tLo ? 1 : 0)
                       + (p8 < tLo ? 1 : 0) + (p12 < tLo ? 1 : 0)
                guard nB >= 3 || nD >= 3 else { continue }

                var vals = [Int](repeating: 0, count: 16)
                for i in 0..<16 {
                    vals[i] = Int(gray[
                        (y + circleOffsets[i].dy) * w
                        + (x + circleOffsets[i].dx)
                    ])
                }

                var maxBright = 0, maxDark = 0
                var runB = 0, runD = 0
                for i in 0..<25 {
                    let v = vals[i % 16]
                    if v > tHi {
                        runB += 1; runD = 0
                        maxBright = max(maxBright, runB)
                    } else if v < tLo {
                        runD += 1; runB = 0
                        maxDark = max(maxDark, runD)
                    } else {
                        runB = 0; runD = 0
                    }
                }

                guard maxBright >= 9 || maxDark >= 9 else { continue }

                var score: Float = 0
                for v in vals {
                    let diff = abs(v - center) - threshold
                    if diff > 0 { score += Float(diff) }
                }

                keypoints.append(Keypoint(
                    x: x, y: y, score: score, angle: 0
                ))
            }
        }
        return keypoints
    }

    // MARK: - Non-Maximum Suppression (grid)

    private static func nonMaxSuppression(
        _ kps: [Keypoint], w: Int, h: Int, cellSize: Int
    ) -> [Keypoint] {
        let gw = (w + cellSize - 1) / cellSize
        let gh = (h + cellSize - 1) / cellSize
        var grid = [Int?](repeating: nil, count: gw * gh)

        for (i, kp) in kps.enumerated() {
            let gi = (kp.y / cellSize) * gw + (kp.x / cellSize)
            if let prev = grid[gi] {
                if kp.score > kps[prev].score { grid[gi] = i }
            } else {
                grid[gi] = i
            }
        }

        return grid.compactMap { idx in idx.map { kps[$0] } }
    }

    // MARK: - Orientation (intensity centroid, ORB method)

    private static let patchRadius = 18

    private static func computeOrientation(
        _ gray: [UInt8], w: Int, h: Int, kp: Keypoint
    ) -> Float {
        var m10 = 0, m01 = 0
        let r = patchRadius
        let r2 = r * r

        for dy in -r...r {
            let dy2 = dy * dy
            let rowOff = (kp.y + dy) * w + kp.x
            for dx in -r...r {
                guard dx * dx + dy2 <= r2 else { continue }
                let val = Int(gray[rowOff + dx])
                m10 += dx * val
                m01 += dy * val
            }
        }

        return atan2(Float(m01), Float(m10))
    }

    // MARK: - Rotated BRIEF Descriptor (256-bit)

    private static let briefPairs: [(Int, Int, Int, Int)] = {
        var s: UInt32 = 0x4F6A_2B3C
        func next() -> Int {
            s = s &* 1_664_525 &+ 1_013_904_223
            return Int(Int16(bitPattern: UInt16(s >> 16))) % 13
        }
        return (0..<256).map { _ in
            (next(), next(), next(), next())
        }
    }()

    private static func computeDescriptor(
        _ gray: [UInt8], w: Int, h: Int, kp: Keypoint
    ) -> BinaryDescriptor {
        let cosA = cos(kp.angle)
        let sinA = sin(kp.angle)
        var bits: (UInt64, UInt64, UInt64, UInt64) = (0, 0, 0, 0)
        let maxX = w - 1, maxY = h - 1

        for i in 0..<256 {
            let (x1, y1, x2, y2) = briefPairs[i]
            let rx1 = min(maxX, max(0, kp.x + Int((cosA * Float(x1) - sinA * Float(y1)).rounded())))
            let ry1 = min(maxY, max(0, kp.y + Int((sinA * Float(x1) + cosA * Float(y1)).rounded())))
            let rx2 = min(maxX, max(0, kp.x + Int((cosA * Float(x2) - sinA * Float(y2)).rounded())))
            let ry2 = min(maxY, max(0, kp.y + Int((sinA * Float(x2) + cosA * Float(y2)).rounded())))

            if gray[ry1 * w + rx1] < gray[ry2 * w + rx2] {
                let bit: UInt64 = 1 << (i & 63)
                switch i >> 6 {
                case 0: bits.0 |= bit
                case 1: bits.1 |= bit
                case 2: bits.2 |= bit
                default: bits.3 |= bit
                }
            }
        }

        return BinaryDescriptor(
            d0: bits.0, d1: bits.1, d2: bits.2, d3: bits.3
        )
    }

    // MARK: - Hamming Distance

    static func hammingDistance(
        _ a: BinaryDescriptor, _ b: BinaryDescriptor
    ) -> Int {
        (a.d0 ^ b.d0).nonzeroBitCount
        + (a.d1 ^ b.d1).nonzeroBitCount
        + (a.d2 ^ b.d2).nonzeroBitCount
        + (a.d3 ^ b.d3).nonzeroBitCount
    }

    // MARK: - Helpers

    private static func bestMatches(
        from src: [BinaryDescriptor],
        to dst: [BinaryDescriptor]
    ) -> [Int] {
        src.map { sd in
            var best = Int.max
            var bestIdx = -1
            for (j, dd) in dst.enumerated() {
                let d = hammingDistance(sd, dd)
                if d < best { best = d; bestIdx = j }
            }
            return bestIdx
        }
    }

    private static func boxBlur3x3(
        _ gray: [UInt8], w: Int, h: Int
    ) -> [UInt8] {
        var out = gray
        for y in 1..<(h - 1) {
            let rowAbove = (y - 1) * w
            let rowCenter = y * w
            let rowBelow = (y + 1) * w
            for x in 1..<(w - 1) {
                let sum = Int(gray[rowAbove + x - 1])
                    + Int(gray[rowAbove + x])
                    + Int(gray[rowAbove + x + 1])
                    + Int(gray[rowCenter + x - 1])
                    + Int(gray[rowCenter + x])
                    + Int(gray[rowCenter + x + 1])
                    + Int(gray[rowBelow + x - 1])
                    + Int(gray[rowBelow + x])
                    + Int(gray[rowBelow + x + 1])
                out[rowCenter + x] = UInt8(sum / 9)
            }
        }
        return out
    }
}
