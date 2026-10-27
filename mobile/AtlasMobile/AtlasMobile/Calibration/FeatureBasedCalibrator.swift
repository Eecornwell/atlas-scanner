import Foundation
import UIKit
import simd

/// RGB feature-based extrinsic refinement.
///
/// Pipeline:
///   1. Slice ERP into perspective view matching iPhone FOV (bilinear)
///   2. FAST-9 corner detection on iPhone image
///   3. NCC patch correlation to find matches in ERP slice
///   4. Iterative outlier rejection: optimize → remove worst → repeat
///   5. Final Nelder-Mead on inliers (rotation only, translation pinned)
///
/// Uses normalized cross-correlation instead of ORB descriptors because
/// cross-camera matching (iPhone vs Insta360) defeats binary descriptors
/// even with color correction — NCC is robust to residual intensity diffs.
enum FeatureBasedCalibrator {

    struct CalibrationResult {
        let extrinsic: RigidTransform
        let matchCount: Int
        let cost: Float
        let iphoneKPCount: Int
        let sliceKPCount: Int
        let iphonePreview: UIImage?
        let slicePreview: UIImage?
    }

    // MARK: - Public API

    struct MatchFrameResult {
        let samples: [ExtrinsicOptimizer.MatchSample]
        let iphoneKPCount: Int
        let nccMatchCount: Int
        let iphonePreview: UIImage?
        let slicePreview: UIImage?
    }

    static func matchFrame(
        iphoneImage: UIImage,
        erpImage: UIImage,
        seed: RigidTransform,
        frame: CalibrationFrame,
        mask: UIImage? = nil,
        matchSize: Int = 512
    ) -> MatchFrameResult {
        let hasCG = erpImage.cgImage != nil
        let erpSz = erpImage.size
        print("[FeatureCal] ERP \(Int(erpSz.width))"
            + "×\(Int(erpSz.height))"
            + " cgImage=\(hasCG)"
            + " iPhone \(Int(iphoneImage.size.width))"
            + "×\(Int(iphoneImage.size.height))")

        func empty(_ msg: String) -> MatchFrameResult {
            print("[FeatureCal] matchFrame: \(msg)")
            return MatchFrameResult(
                samples: [], iphoneKPCount: 0,
                nccMatchCount: 0,
                iphonePreview: nil, slicePreview: nil
            )
        }

        guard let slice = sliceERPToIPhoneFOV(
            erpImage: erpImage,
            extrinsic: seed,
            frame: frame,
            outputSize: matchSize
        ) else {
            return empty("failed to create ERP slice")
        }

        let iphoneResized = resizeImage(
            iphoneImage, width: slice.w, height: slice.h
        )
        guard let iphoneResized else {
            return empty("failed to resize iPhone image")
        }

        print("[FeatureCal] matching at \(slice.w)×\(slice.h)")

        guard let iGray = FeatureDetector.toGrayscale(
            iphoneResized
        ),
            let sGray = FeatureDetector.toGrayscale(
                slice.image
            )
        else {
            return empty("grayscale conversion failed")
        }

        let iFeats = FeatureDetector.detectAndDescribe(
            gray: iGray.pixels, width: iGray.width,
            height: iGray.height,
            maxKeypoints: 2000, fastThreshold: 12
        )

        print("[FeatureCal] iPhone corners:"
            + " \(iFeats.keypoints.count)")

        guard iFeats.keypoints.count >= 20 else {
            return MatchFrameResult(
                samples: [],
                iphoneKPCount: iFeats.keypoints.count,
                nccMatchCount: 0,
                iphonePreview: iphoneResized,
                slicePreview: slice.image
            )
        }

        let nccMatches = correlationMatch(
            grayA: iGray.pixels,
            wA: iGray.width, hA: iGray.height,
            grayB: sGray.pixels,
            wB: sGray.width, hB: sGray.height,
            keypoints: iFeats.keypoints,
            searchRadius: 32,
            patchHalf: 10,
            minNCC: 0.4
        )

        print("[FeatureCal] NCC matches: \(nccMatches.count)"
            + " (of \(iFeats.keypoints.count) corners)")

        let maskBool: [Bool]?
        if let maskCG = mask?.cgImage {
            let mw = Int(erpImage.size.width)
            let mh = Int(erpImage.size.height)
            maskBool = extractGrayMask(
                from: maskCG, width: mw, height: mh
            )
        } else {
            maskBool = nil
        }

        let samples = buildSamplesFromNCC(
            matches: nccMatches,
            iKeypoints: iFeats.keypoints,
            slice: slice,
            frame: frame,
            erpImage: erpImage,
            mask: maskBool
        )

        print("[FeatureCal] \(samples.count) samples")

        return MatchFrameResult(
            samples: samples,
            iphoneKPCount: iFeats.keypoints.count,
            nccMatchCount: nccMatches.count,
            iphonePreview: iphoneResized,
            slicePreview: slice.image
        )
    }

    static func calibrate(
        iphoneImage: UIImage,
        erpImage: UIImage,
        seed: RigidTransform,
        frame: CalibrationFrame,
        mask: UIImage? = nil
    ) -> CalibrationResult {
        let mr = matchFrame(
            iphoneImage: iphoneImage,
            erpImage: erpImage,
            seed: seed,
            frame: frame,
            mask: mask
        )

        guard mr.samples.count >= 6 else {
            return CalibrationResult(
                extrinsic: seed, matchCount: 0,
                cost: .infinity,
                iphoneKPCount: mr.iphoneKPCount,
                sliceKPCount: mr.nccMatchCount,
                iphonePreview: mr.iphonePreview,
                slicePreview: mr.slicePreview
            )
        }

        let erpW = Int(erpImage.size.width)
        let erpH = Int(erpImage.size.height)
        var samples = mr.samples

        var currentSeed = seed
        for pass in 0..<3 {
            guard samples.count >= 4 else { break }

            let (refined, cost) = ExtrinsicOptimizer.optimize(
                samples: samples,
                seed: currentSeed,
                erpW: erpW,
                erpH: erpH,
                maxIterations: pass == 2 ? 800 : 400
            )
            currentSeed = refined

            if pass < 2 {
                let residuals = computeResiduals(
                    samples: samples,
                    extrinsic: refined,
                    erpW: erpW, erpH: erpH
                )

                let sorted = residuals.sorted()
                let medianR = sorted[sorted.count / 2]
                let threshold = max(medianR * 3, 400)

                let before = samples.count
                samples = zip(samples, residuals)
                    .filter { $0.1 <= threshold }
                    .map { $0.0 }

                print("[FeatureCal] pass \(pass):"
                    + " cost=\(String(format: "%.1f", cost))"
                    + " median="
                    + "\(String(format: "%.1f", medianR))"
                    + " kept \(samples.count)/\(before)")
            } else {
                print("[FeatureCal] pass \(pass):"
                    + " cost=\(String(format: "%.1f", cost))"
                    + " inliers=\(samples.count)")
            }
        }

        guard samples.count >= 4 else {
            return CalibrationResult(
                extrinsic: seed, matchCount: 0,
                cost: .infinity,
                iphoneKPCount: mr.iphoneKPCount,
                sliceKPCount: mr.nccMatchCount,
                iphonePreview: mr.iphonePreview,
                slicePreview: mr.slicePreview
            )
        }

        let result = RigidTransform(
            roll: currentSeed.roll,
            pitch: currentSeed.pitch,
            yaw: currentSeed.yaw,
            x: seed.x, y: seed.y, z: seed.z
        )

        let finalCost = meanResidual(
            samples: samples, extrinsic: result,
            erpW: erpW, erpH: erpH
        )

        print("[FeatureCal] final RPY="
            + "(\(String(format: "%.3f", result.roll)),"
            + " \(String(format: "%.3f", result.pitch)),"
            + " \(String(format: "%.3f", result.yaw)))"
            + " cost=\(String(format: "%.2f", finalCost))"
            + " inliers=\(samples.count)")

        return CalibrationResult(
            extrinsic: result,
            matchCount: samples.count,
            cost: finalCost,
            iphoneKPCount: mr.iphoneKPCount,
            sliceKPCount: mr.nccMatchCount,
            iphonePreview: mr.iphonePreview,
            slicePreview: mr.slicePreview
        )
    }

    static func optimizeJoint(
        samples: inout [ExtrinsicOptimizer.MatchSample],
        seed: RigidTransform,
        erpW: Int,
        erpH: Int,
        passes: Int = 3,
        finalIterations: Int = 1200
    ) -> RigidTransform {
        var current = seed
        for pass in 0..<passes {
            guard samples.count >= 4 else { break }
            let iters = pass == passes - 1
                ? finalIterations : 500
            let (refined, cost) = ExtrinsicOptimizer.optimize(
                samples: samples,
                seed: current,
                erpW: erpW, erpH: erpH,
                maxIterations: iters
            )
            current = refined

            if pass < passes - 1 {
                let residuals = computeResiduals(
                    samples: samples,
                    extrinsic: refined,
                    erpW: erpW, erpH: erpH
                )
                let sorted = residuals.sorted()
                let medianR = sorted[sorted.count / 2]
                let threshold = max(medianR * 2.5, 200)
                let before = samples.count
                samples = zip(samples, residuals)
                    .filter { $0.1 <= threshold }
                    .map { $0.0 }
                print("[FeatureCal] joint pass \(pass):"
                    + " cost=\(String(format: "%.1f", cost))"
                    + " median="
                    + "\(String(format: "%.1f", medianR))"
                    + " kept \(samples.count)/\(before)")
            } else {
                print("[FeatureCal] joint pass \(pass):"
                    + " cost=\(String(format: "%.1f", cost))"
                    + " inliers=\(samples.count)")
            }
        }
        return RigidTransform(
            roll: current.roll, pitch: current.pitch,
            yaw: current.yaw,
            x: seed.x, y: seed.y, z: seed.z
        )
    }

    static func jointCost(
        samples: [ExtrinsicOptimizer.MatchSample],
        extrinsic: RigidTransform,
        erpW: Int,
        erpH: Int
    ) -> Float {
        return meanResidual(
            samples: samples, extrinsic: extrinsic,
            erpW: erpW, erpH: erpH
        )
    }

    // MARK: - NCC Patch Correlation

    struct NCCMatch {
        let queryIdx: Int
        let matchX: Float
        let matchY: Float
        let score: Float
    }

    private static func correlationMatch(
        grayA: [UInt8], wA: Int, hA: Int,
        grayB: [UInt8], wB: Int, hB: Int,
        keypoints: [Keypoint],
        searchRadius: Int,
        patchHalf: Int,
        minNCC: Float
    ) -> [NCCMatch] {
        var results: [NCCMatch] = []
        results.reserveCapacity(keypoints.count)

        for (qi, kp) in keypoints.enumerated() {
            let px = kp.x, py = kp.y

            // Ensure patch fits in image A
            guard px >= patchHalf, px < wA - patchHalf,
                  py >= patchHalf, py < hA - patchHalf
            else { continue }

            // Extract and normalize patch from image A
            let patchSize = 2 * patchHalf + 1
            let n = patchSize * patchSize
            var patchA = [Float](repeating: 0, count: n)
            var sumA: Float = 0
            for dy in -patchHalf...patchHalf {
                for dx in -patchHalf...patchHalf {
                    let val = Float(grayA[(py + dy) * wA + (px + dx)])
                    let idx = (dy + patchHalf) * patchSize
                        + (dx + patchHalf)
                    patchA[idx] = val
                    sumA += val
                }
            }
            let meanA = sumA / Float(n)

            var varA: Float = 0
            for i in 0..<n {
                patchA[i] -= meanA
                varA += patchA[i] * patchA[i]
            }
            let stdA = sqrt(varA)
            guard stdA > 1e-6 else { continue }

            // Search in image B
            let xLo = max(patchHalf, px - searchRadius)
            let xHi = min(wB - patchHalf - 1, px + searchRadius)
            let yLo = max(patchHalf, py - searchRadius)
            let yHi = min(hB - patchHalf - 1, py + searchRadius)

            guard xLo <= xHi, yLo <= yHi else { continue }

            var bestNCC: Float = -1
            var bestX = px, bestY = py

            for sy in yLo...yHi {
                for sx in xLo...xHi {
                    var sumB: Float = 0
                    for dy in -patchHalf...patchHalf {
                        for dx in -patchHalf...patchHalf {
                            sumB += Float(
                                grayB[(sy + dy) * wB + (sx + dx)]
                            )
                        }
                    }
                    let meanB = sumB / Float(n)

                    var cross: Float = 0
                    var varB: Float = 0
                    for dy in -patchHalf...patchHalf {
                        for dx in -patchHalf...patchHalf {
                            let bVal = Float(
                                grayB[(sy + dy) * wB + (sx + dx)]
                            ) - meanB
                            let idx = (dy + patchHalf) * patchSize
                                + (dx + patchHalf)
                            cross += patchA[idx] * bVal
                            varB += bVal * bVal
                        }
                    }

                    let denom = stdA * sqrt(varB)
                    guard denom > 1e-6 else { continue }
                    let ncc = cross / denom

                    if ncc > bestNCC {
                        bestNCC = ncc
                        bestX = sx
                        bestY = sy
                    }
                }
            }

            if bestNCC >= minNCC {
                var subX = Float(bestX)
                var subY = Float(bestY)

                let computeNCC = { (sx: Int, sy: Int) -> Float in
                    guard sx >= patchHalf,
                          sx < wB - patchHalf,
                          sy >= patchHalf,
                          sy < hB - patchHalf
                    else { return -1 }
                    var sB: Float = 0
                    for dy2 in -patchHalf...patchHalf {
                        for dx2 in -patchHalf...patchHalf {
                            sB += Float(
                                grayB[(sy + dy2) * wB
                                       + (sx + dx2)]
                            )
                        }
                    }
                    let mB = sB / Float(n)
                    var cr2: Float = 0, vB: Float = 0
                    for dy2 in -patchHalf...patchHalf {
                        for dx2 in -patchHalf...patchHalf {
                            let bv = Float(
                                grayB[(sy + dy2) * wB
                                       + (sx + dx2)]
                            ) - mB
                            let i2 = (dy2 + patchHalf)
                                * patchSize
                                + (dx2 + patchHalf)
                            cr2 += patchA[i2] * bv
                            vB += bv * bv
                        }
                    }
                    let d = stdA * sqrt(vB)
                    return d > 1e-6 ? cr2 / d : -1
                }

                let nL = computeNCC(bestX - 1, bestY)
                let nR = computeNCC(bestX + 1, bestY)
                let denom_x = nL - 2 * bestNCC + nR
                if abs(denom_x) > 1e-8 {
                    let dx = 0.5 * (nL - nR) / denom_x
                    if abs(dx) < 1 { subX += dx }
                }

                let nU = computeNCC(bestX, bestY - 1)
                let nD = computeNCC(bestX, bestY + 1)
                let denom_y = nU - 2 * bestNCC + nD
                if abs(denom_y) > 1e-8 {
                    let dy = 0.5 * (nU - nD) / denom_y
                    if abs(dy) < 1 { subY += dy }
                }

                results.append(NCCMatch(
                    queryIdx: qi,
                    matchX: subX, matchY: subY,
                    score: bestNCC
                ))
            }
        }

        return results
    }

    // MARK: - ERP perspective slice (bilinear)

    private struct Slice {
        let image: UIImage
        let lonMap: [Float]
        let latMap: [Float]
        let w: Int
        let h: Int
    }

    private static func sliceERPToIPhoneFOV(
        erpImage: UIImage,
        extrinsic: RigidTransform,
        frame: CalibrationFrame,
        outputSize: Int
    ) -> Slice? {
        guard let erpCG = erpImage.cgImage else {
            print("[FeatureCal] slice: erpImage has no cgImage")
            return nil
        }
        let erpW = erpCG.width, erpH = erpCG.height
        print("[FeatureCal] slice: ERP \(erpW)×\(erpH) → \(outputSize)px")

        let di = frame.depthIntrinsics()
        let sX = Float(frame.imageW) / Float(frame.depthW)
        let sY = Float(frame.imageH) / Float(frame.depthH)

        let aspect = Float(frame.imageW) / Float(frame.imageH)
        let outW = outputSize
        let outH = max(1, Int(Float(outputSize) / aspect))

        let outFx = di.fx * sX * Float(outW) / Float(frame.imageW)
        let outFy = di.fy * sY * Float(outH) / Float(frame.imageH)
        let outCx = di.cx * sX * Float(outW) / Float(frame.imageW)
        let outCy = di.cy * sY * Float(outH) / Float(frame.imageH)

        let T = ExtrinsicOptimizer.seedToCameraMatrix(extrinsic)
        let R = simd_float3x3(
            SIMD3(T.columns.0.x, T.columns.0.y, T.columns.0.z),
            SIMD3(T.columns.1.x, T.columns.1.y, T.columns.1.z),
            SIMD3(T.columns.2.x, T.columns.2.y, T.columns.2.z)
        )

        guard let erpPixels = extractRGBA(from: erpCG) else {
            return nil
        }

        let n = outW * outH
        var lonMap = [Float](repeating: 0, count: n)
        var latMap = [Float](repeating: 0, count: n)
        var outPixels = [UInt8](repeating: 255, count: n * 4)

        let eWf = Float(erpW), eHf = Float(erpH)

        for row in 0..<outH {
            for col in 0..<outW {
                // Flip 180° so the slice matches the iPhone UIImage,
                // which is vertically flipped from intrinsic convention
                // due to CIImage→CGImage coordinate inversion.
                let lCol = outW - 1 - col
                let lRow = outH - 1 - row
                let xCam = (Float(lCol) - outCx) / outFx
                let yCam = (Float(lRow) - outCy) / outFy
                let ray = SIMD3<Float>(-xCam, -yCam, 1.0)

                let rotated = simd_normalize(R * ray)
                let lat = -asin(max(-1, min(1, rotated.y)))
                let lon = atan2(rotated.x, rotated.z)

                let idx = row * outW + col
                lonMap[idx] = lon
                latMap[idx] = lat

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

                let i00 = (v0 * erpW + u0) * 4
                let i10 = (v0 * erpW + u1) * 4
                let i01 = (v1 * erpW + u0) * 4
                let i11 = (v1 * erpW + u1) * 4

                let dstI = idx * 4
                for c in 0..<3 {
                    let p00 = Float(erpPixels[i00 + c])
                    let p10 = Float(erpPixels[i10 + c])
                    let p01 = Float(erpPixels[i01 + c])
                    let p11 = Float(erpPixels[i11 + c])
                    let top = p00 + (p10 - p00) * fu
                    let bot = p01 + (p11 - p01) * fu
                    let val = top + (bot - top) * fv
                    outPixels[dstI + c] = UInt8(max(0, min(255, val)))
                }
                outPixels[dstI + 3] = 255
            }
        }

        guard let ctx = CGContext(
            data: &outPixels, width: outW, height: outH,
            bitsPerComponent: 8, bytesPerRow: outW * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ), let cg = ctx.makeImage() else { return nil }

        return Slice(
            image: UIImage(cgImage: cg),
            lonMap: lonMap, latMap: latMap,
            w: outW, h: outH
        )
    }

    // MARK: - Sample construction from NCC matches

    private static func buildSamplesFromNCC(
        matches: [NCCMatch],
        iKeypoints: [Keypoint],
        slice: Slice,
        frame: CalibrationFrame,
        erpImage: UIImage,
        mask: [Bool]?
    ) -> [ExtrinsicOptimizer.MatchSample] {
        let di = frame.depthIntrinsics()
        let sX = Float(frame.imageW) / Float(frame.depthW)
        let sY = Float(frame.imageH) / Float(frame.depthH)
        let imgFx = di.fx * sX
        let imgFy = di.fy * sY
        let imgCx = di.cx * sX
        let imgCy = di.cy * sY

        let sliceToImgX = Float(frame.imageW) / Float(slice.w)
        let sliceToImgY = Float(frame.imageH) / Float(slice.h)

        let erpW = Float(erpImage.size.width)
        let erpH = Float(erpImage.size.height)
        let erpWi = Int(erpW)

        var samples: [ExtrinsicOptimizer.MatchSample] = []
        samples.reserveCapacity(matches.count)

        for m in matches {
            let iKp = iKeypoints[m.queryIdx]

            // iPhone pixel → camera bearing (unfliip 180° to get
            // intrinsic-convention coordinates from flipped image)
            let imgX = Float(slice.w - 1 - iKp.x) * sliceToImgX
            let imgY = Float(slice.h - 1 - iKp.y) * sliceToImgY
            let xCam = (imgX - imgCx) / imgFx
            let yCam = (imgY - imgCy) / imgFy
            let bearing = simd_normalize(
                SIMD3<Float>(-xCam, -yCam, 1.0)
            )

            // NCC match position in slice → lon/lat (bilinear)
            let mx = m.matchX, my = m.matchY
            let mx0 = max(0, min(slice.w - 1, Int(mx)))
            let my0 = max(0, min(slice.h - 1, Int(my)))
            let mx1 = min(mx0 + 1, slice.w - 1)
            let my1 = min(my0 + 1, slice.h - 1)
            let fx = mx - Float(mx0)
            let fy = my - Float(my0)
            let i00 = my0 * slice.w + mx0
            let i10 = my0 * slice.w + mx1
            let i01 = my1 * slice.w + mx0
            let i11 = my1 * slice.w + mx1
            guard i11 < slice.lonMap.count else { continue }
            let lon = slice.lonMap[i00] * (1 - fx) * (1 - fy)
                + slice.lonMap[i10] * fx * (1 - fy)
                + slice.lonMap[i01] * (1 - fx) * fy
                + slice.lonMap[i11] * fx * fy
            let lat = slice.latMap[i00] * (1 - fx) * (1 - fy)
                + slice.latMap[i10] * fx * (1 - fy)
                + slice.latMap[i01] * (1 - fx) * fy
                + slice.latMap[i11] * fx * fy

            let eU = erpW * (0.5 + lon / (2 * .pi))
            let eV = erpH * (0.5 - lat / .pi)

            if let mk = mask {
                let mCol = ((Int(eU) % erpWi) + erpWi) % erpWi
                let mRow = max(0, min(Int(erpH) - 1, Int(eV)))
                if mk[mRow * erpWi + mCol] { continue }
            }

            samples.append(ExtrinsicOptimizer.MatchSample(
                lidarBearing: bearing,
                instaU: eU,
                instaV: eV
            ))
        }

        return samples
    }

    // MARK: - Residual computation

    private static func computeResiduals(
        samples: [ExtrinsicOptimizer.MatchSample],
        extrinsic: RigidTransform,
        erpW: Int,
        erpH: Int
    ) -> [Float] {
        let T = ExtrinsicOptimizer.seedToCameraMatrix(extrinsic)
        let R = simd_float3x3(
            SIMD3(T.columns.0.x, T.columns.0.y, T.columns.0.z),
            SIMD3(T.columns.1.x, T.columns.1.y, T.columns.1.z),
            SIMD3(T.columns.2.x, T.columns.2.y, T.columns.2.z)
        )
        let W = Float(erpW), H = Float(erpH)

        return samples.map { s in
            let b = simd_normalize(R * s.lidarBearing)
            let lat = -asin(max(-1, min(1, b.y)))
            let lon = atan2(b.x, b.z)
            let u = W * (0.5 + lon / (2 * .pi))
            let v = H * (0.5 - lat / .pi)

            var du = u - s.instaU
            if abs(du) > W / 2 { du -= copysign(W, du) }
            let dv = v - s.instaV
            return du * du + dv * dv
        }
    }

    private static func meanResidual(
        samples: [ExtrinsicOptimizer.MatchSample],
        extrinsic: RigidTransform,
        erpW: Int,
        erpH: Int
    ) -> Float {
        let residuals = computeResiduals(
            samples: samples, extrinsic: extrinsic,
            erpW: erpW, erpH: erpH
        )
        guard !residuals.isEmpty else { return .infinity }
        return residuals.reduce(0, +) / Float(residuals.count)
    }

    // MARK: - Helpers

    private static func resizeImage(
        _ image: UIImage, width: Int, height: Int
    ) -> UIImage? {
        let size = CGSize(width: width, height: height)
        let fmt = UIGraphicsImageRendererFormat()
        fmt.scale = 1.0
        let renderer = UIGraphicsImageRenderer(size: size, format: fmt)
        return renderer.image { _ in
            image.draw(in: CGRect(origin: .zero, size: size))
        }
    }

    private static func extractRGBA(
        from cgImage: CGImage
    ) -> [UInt8]? {
        let w = cgImage.width, h = cgImage.height
        var pixels = [UInt8](repeating: 0, count: w * h * 4)
        guard let ctx = CGContext(
            data: &pixels, width: w, height: h,
            bitsPerComponent: 8, bytesPerRow: w * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return nil }
        ctx.draw(
            cgImage,
            in: CGRect(x: 0, y: 0, width: w, height: h)
        )
        return pixels
    }

    private static func extractGrayMask(
        from cgImage: CGImage, width: Int, height: Int
    ) -> [Bool] {
        var gray = [UInt8](repeating: 0, count: width * height)
        guard let ctx = CGContext(
            data: &gray, width: width, height: height,
            bitsPerComponent: 8, bytesPerRow: width,
            space: CGColorSpaceCreateDeviceGray(),
            bitmapInfo: CGImageAlphaInfo.none.rawValue
        ) else {
            return [Bool](repeating: false, count: width * height)
        }
        ctx.draw(
            cgImage,
            in: CGRect(x: 0, y: 0, width: width, height: height)
        )
        return gray.map { $0 > 128 }
    }
}
