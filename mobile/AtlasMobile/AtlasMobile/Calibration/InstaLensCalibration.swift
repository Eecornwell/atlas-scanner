import Foundation
import simd

/// The Insta360 factory lens calibration stored in each photo.
///
/// Every X-series INSP ends with an Insta360 trailer. Its INFO record (a
/// protobuf) holds the camera's own calibration in the `offset` string of
/// fields 111 and 112; fields 4 to 6 hold older forms. Version 6 states each
/// lens with the unified-sphere (Mei) model and a 13-term radial, tangential
/// and thin-prism distortion, version 3 with 5 Brown terms. The format and
/// the projection follow telemetry-parser (src/insta360/rig.rs, MIT) and
/// rooster-vehicle's tools/capture_export/insv_calib.py, which reads the same
/// record from X6 video.
///
/// Frames are x right, y down, z forward. The front lens is lens 0, the left
/// half of the photo, and its frame is the panorama frame the tiles and the
/// ERP are rendered in. The rear lens is lens 1, the right half.
///
/// On X5 IAHEA26019RESN the nominal equidistant model the stitcher used
/// before was 1-6 deg off in each lens, and front and rear disagreed by about
/// 12 deg where they overlap.
struct InstaLensCalibration {
    /// One lens, in the pixels of the sensor window the camera records.
    struct Lens {
        let xi: Float
        let fx: Float
        let fy: Float
        /// Principal point inside this lens's window.
        let cx: Float
        let cy: Float
        /// 13 distortion terms (offset version 6) or 5 (version 3).
        let d: [Float]
        /// Size of the window the values above are stated in.
        let windowW: Float
        let windowH: Float
        /// Rays further than this from the axis are outside the lens.
        let fieldLimitDeg: Float
    }

    let front: Lens
    let rear: Lens
    /// Rear lens frame from front lens frame (camera 1 from camera 0).
    let rearFromFront: simd_float3x3
    /// Camera, serial and offset version, for logs.
    let summary: String

    /// The lens as seen in a decoded dual-fisheye buffer of this size
    /// (both lenses side by side, any scale of the photo).
    func pixelLens(
        rear isRear: Bool, bufferWidth: Int, bufferHeight: Int
    ) -> InstaPixelLens {
        InstaPixelLens(
            isRear ? rear : front, index: isRear ? 1 : 0,
            bufferWidth: bufferWidth, bufferHeight: bufferHeight)
    }

    // MARK: - Loading

    /// The calibration in an INSP file, or nil if the file has none this
    /// code can read (no Insta360 trailer, an unknown offset version, a
    /// mirrored lens, or values that don't fit a two-lens photo).
    static func load(url: URL) -> InstaLensCalibration? {
        guard let info = readInfoRecord(url: url) else { return nil }
        return parse(info: info)
    }

    /// Whether the file ends with an Insta360 trailer, as a complete INSP
    /// does. A download cut short doesn't.
    static func hasTrailer(url: URL) -> Bool {
        guard let fh = try? FileHandle(forReadingFrom: url) else {
            return false
        }
        defer { try? fh.close() }
        guard let size = try? fh.seekToEnd(), size > 72,
              (try? fh.seek(toOffset: size - 32)) != nil,
              let tail = try? fh.read(upToCount: 32)
        else { return false }
        return [UInt8](tail) == magic
    }

    private static let magic = Array("8db42d694ccc418790edff439fe026bf".utf8)
    private static let infoRecordId: UInt16 = 0x101
    /// Lens types with a 100 deg field limit (rooster insv_calib.py);
    /// other types get 95 deg.
    private static let wideLensTypes: Set<Int> = [
        19, 33, 38, 40, 41, 44, 70, 71, 85, 113, 131, 142, 149, 150, 193,
    ]

    /// Bytes of the trailer's INFO record. The trailer ends with a 72-byte
    /// footer: its last 32 bytes are the magic, and the 4 bytes 40 from
    /// the end give the trailer's size. Before the footer, each record is
    /// its data followed by a 6-byte header (UInt16 id, UInt32 size).
    static func readInfoRecord(url: URL) -> [UInt8]? {
        guard let fh = try? FileHandle(forReadingFrom: url) else {
            return nil
        }
        defer { try? fh.close() }
        guard let size = try? fh.seekToEnd(), size > 72,
              (try? fh.seek(toOffset: size - 40)) != nil,
              let tailData = try? fh.read(upToCount: 40)
        else { return nil }
        let tail = [UInt8](tailData)
        guard tail.count == 40, Array(tail[8..<40]) == magic else {
            return nil
        }
        let total = UInt64(u32(tail, 0))
        guard total > 72, total <= size else { return nil }
        let start = size - total
        var end = size - 72
        while end > start + 6 {
            guard (try? fh.seek(toOffset: end - 6)) != nil,
                  let hd = try? fh.read(upToCount: 6), hd.count == 6
            else { return nil }
            let h = [UInt8](hd)
            let id = UInt16(h[0]) | UInt16(h[1]) << 8
            let len = UInt64(u32(h, 2))
            if len == 0 || len > end - 6 - start { return nil }
            if id == infoRecordId {
                guard (try? fh.seek(toOffset: end - 6 - len)) != nil,
                      let rec = try? fh.read(upToCount: Int(len)),
                      rec.count == Int(len)
                else { return nil }
                return [UInt8](rec)
            }
            end -= 6 + len
        }
        return nil
    }

    /// Little-endian UInt32 at b[i].
    private static func u32(_ b: [UInt8], _ i: Int) -> UInt32 {
        let lo = UInt32(b[i]) | UInt32(b[i + 1]) << 8
        let hi = UInt32(b[i + 2]) << 16 | UInt32(b[i + 3]) << 24
        return lo | hi
    }

    /// One field value of a protobuf message.
    private enum ProtoValue {
        case varint(UInt64)
        case bytes([UInt8])
        case fixed
    }

    /// Top-level fields of a protobuf message, or nil if malformed.
    private static func protoFields(
        _ b: [UInt8]
    ) -> [Int: [ProtoValue]]? {
        var out: [Int: [ProtoValue]] = [:]
        var i = 0
        func varint() -> UInt64? {
            var r: UInt64 = 0
            var shift: UInt64 = 0
            while i < b.count, shift < 64 {
                let c = b[i]
                i += 1
                r |= UInt64(c & 0x7F) << shift
                if c < 0x80 { return r }
                shift += 7
            }
            return nil
        }
        while i < b.count {
            guard let key = varint() else { return nil }
            let field = Int(key >> 3)
            let value: ProtoValue
            switch key & 7 {
            case 0:
                guard let v = varint() else { return nil }
                value = .varint(v)
            case 1:
                guard i + 8 <= b.count else { return nil }
                i += 8
                value = .fixed
            case 2:
                guard let n = varint(), n <= UInt64(b.count - i)
                else { return nil }
                value = .bytes(Array(b[i..<i + Int(n)]))
                i += Int(n)
            case 5:
                guard i + 4 <= b.count else { return nil }
                i += 4
                value = .fixed
            default:
                return nil
            }
            out[field, default: []].append(value)
        }
        return out
    }

    private static func bytesField(
        _ f: [Int: [ProtoValue]], _ n: Int
    ) -> [[UInt8]] {
        var out: [[UInt8]] = []
        for v in f[n] ?? [] {
            if case .bytes(let b) = v { out.append(b) }
        }
        return out
    }

    private static func intField(
        _ f: [Int: [ProtoValue]], _ n: Int
    ) -> Double {
        for v in f[n] ?? [] {
            if case .varint(let x) = v {
                return Double(Int64(bitPattern: x))
            }
        }
        return 0
    }

    private struct OffsetLens {
        let v: [Double]
        let nd: Int
        var xi: Double { v[0] }
        var fx: Double { v[1] }
        var fy: Double { v[2] }
        var cx: Double { v[3] }
        var cy: Double { v[4] }
        var ypr: (Double, Double, Double) { (v[5], v[6], v[7]) }
        var d: [Double] { Array(v[11..<11 + nd]) }
        var w: Double { v[11 + nd] }
        var h: Double { v[12 + nd] }
        var lensType: Int { Int(v[13 + nd]) }
    }

    /// offset = <lens count>_<fields per lens>..._<tail>, where
    /// tail = (version << 16) | (X << 8); X = 5 means lens 0 is mirrored.
    /// Per lens: xi fx fy cx cy yaw pitch roll tx ty tz d0..dn W H type,
    /// in pixels of the side-by-side calibration frame (W = both lenses).
    private static func parseOffset(
        _ s: String
    ) -> (version: Int, mirrored: Bool, lenses: [OffsetLens])? {
        let tok = s.split(separator: "_", omittingEmptySubsequences: false)
            .map { $0.trimmingCharacters(in: .whitespaces) }
        guard tok.count >= 3, let tail = Int(tok[tok.count - 1]),
              let n = Int(tok[0]), n >= 1
        else { return nil }
        let version = tail >> 16
        let x = (tail >> 8) & 0xFF
        let per: Int
        switch version {
        case 6: per = 27
        case 3: per = 19
        default: return nil
        }
        guard tok.count == n * per + 2 else { return nil }
        let nd = version == 6 ? 13 : 5
        var lenses: [OffsetLens] = []
        for i in 0..<n {
            var vals: [Double] = []
            for t in tok[(1 + i * per)..<(1 + (i + 1) * per)] {
                guard let v = Double(t) else { return nil }
                vals.append(v)
            }
            lenses.append(OffsetLens(v: vals, nd: nd))
        }
        return (version, x == 5, lenses)
    }

    static func parse(info: [UInt8]) -> InstaLensCalibration? {
        guard let f = protoFields(info) else { return nil }
        func text(_ n: Int) -> String {
            bytesField(f, n).first.map {
                String(decoding: $0, as: UTF8.self)
            } ?? ""
        }
        // Newer strings first; the highest version wins (as insv_calib.py).
        var best: (version: Int, mirrored: Bool, lenses: [OffsetLens])?
        for n in [111, 112, 4, 5, 6] {
            for b in bytesField(f, n) {
                guard let o = parseOffset(
                    String(decoding: b, as: UTF8.self))
                else { continue }
                if best == nil || o.version > best!.version { best = o }
            }
        }
        guard let off = best, !off.mirrored, off.lenses.count == 2
        else { return nil }
        let l0 = off.lenses[0]
        let frameW = l0.w / 2
        let frameH = l0.h
        guard frameW > 0, frameH > 0 else { return nil }

        // window_crop_info: the sensor window the string describes and the
        // part the camera recorded (rooster insv_calib.py). Photos record
        // the whole window.
        var srcW = frameW, srcH = frameH, dstW = frameW, dstH = frameH
        var ox = 0.0, oy = 0.0
        if let cropBytes = bytesField(f, 27).first,
           let crop = protoFields(cropBytes)
        {
            let sw = intField(crop, 1), sh = intField(crop, 2)
            let dw = intField(crop, 3), dh = intField(crop, 4)
            if sw > 0, sh > 0, dw > 0, dh > 0,
               abs(frameW * sh - frameH * sw) < 0.5
            {
                srcW = sw; srcH = sh; dstW = dw; dstH = dh
                ox = intField(crop, 5); oy = intField(crop, 6)
            }
        }
        let scale = srcW / frameW

        var lenses: [Lens] = []
        var rotations: [simd_double3x3] = []
        for (i, l) in off.lenses.enumerated() {
            // Lens i is half i of the calibration frame.
            let localCx = l.cx - frameW * Double(i)
            guard l.xi >= 0, l.fx > 0, l.fy > 0,
                  localCx > 0, localCx < frameW,
                  l.cy > 0, l.cy < frameH
            else { return nil }
            let cx = scale * (localCx + 0.5) - 0.5
                - (srcW - dstW) * 0.5 - ox
            let cy = scale * (l.cy + 0.5) - 0.5
                - (srcH - dstH) * 0.5 - oy
            lenses.append(Lens(
                xi: Float(l.xi),
                fx: Float(l.fx * scale), fy: Float(l.fy * scale),
                cx: Float(cx), cy: Float(cy),
                d: l.d.map { Float($0) },
                windowW: Float(dstW), windowH: Float(dstH),
                fieldLimitDeg: wideLensTypes.contains(l.lensType)
                    ? 100 : 95))
            rotations.append(vendorRotation(l.ypr, index: i, count: 2))
        }
        // Camera 1 from camera 0; the shared body rotation cancels.
        let r10 = rotations[1] * rotations[0].transpose
        func column(_ c: SIMD3<Double>) -> SIMD3<Float> {
            SIMD3<Float>(Float(c.x), Float(c.y), Float(c.z))
        }
        let rearFromFront = simd_float3x3(
            column(r10.columns.0), column(r10.columns.1),
            column(r10.columns.2))
        let calib = InstaLensCalibration(
            front: lenses[0], rear: lenses[1],
            rearFromFront: rearFromFront,
            summary: "\(text(2)) \(text(1)): factory lens calibration"
                + " (offset v\(off.version))")
        return calib.isPlausible() ? calib : nil
    }

    /// Vendor rotation (world to camera): Ry(pitch + 90) Rz(yaw) Rx(roll),
    /// with an extra Rz(180) on lens 0 of a two-lens body.
    private static func vendorRotation(
        _ ypr: (Double, Double, Double), index: Int, count: Int
    ) -> simd_double3x3 {
        let r = Double.pi / 180
        func rx(_ a: Double) -> simd_double3x3 {
            simd_double3x3(rows: [
                SIMD3(1, 0, 0), SIMD3(0, cos(a), -sin(a)),
                SIMD3(0, sin(a), cos(a)),
            ])
        }
        func ry(_ a: Double) -> simd_double3x3 {
            simd_double3x3(rows: [
                SIMD3(cos(a), 0, sin(a)), SIMD3(0, 1, 0),
                SIMD3(-sin(a), 0, cos(a)),
            ])
        }
        func rz(_ a: Double) -> simd_double3x3 {
            simd_double3x3(rows: [
                SIMD3(cos(a), -sin(a), 0), SIMD3(sin(a), cos(a), 0),
                SIMD3(0, 0, 1),
            ])
        }
        var m = ry((ypr.1 + 90) * r) * rz(ypr.0 * r) * rx(ypr.2 * r)
        if index == 0 && count == 2 { m = m * rz(Double.pi) }
        return m
    }

    /// The axis of each lens lands near the middle of its half, the rear
    /// lens faces away from the front one, and a ray 90 deg off the axis
    /// stays inside the lens's half. Guards against a misread record.
    private func isPlausible() -> Bool {
        let w = 2 * Int(front.windowW.rounded())
        let h = Int(front.windowH.rounded())
        for rear in [false, true] {
            let lens = pixelLens(
                rear: rear, bufferWidth: w, bufferHeight: h)
            let half = Float(w) / 2
            let x0 = rear ? half : 0
            guard let c = lens.project(0, 0, 1),
                  abs(c.x - (x0 + half / 2)) < half * 0.1,
                  abs(c.y - Float(h) / 2) < Float(h) * 0.1,
                  lens.project(1, 0, 0) != nil,
                  lens.project(0, 1, 0) != nil
            else { return false }
        }
        return rearFromFront.columns.2.z < -0.9
    }
}

/// One lens of an InstaLensCalibration in the pixels of a decoded
/// dual-fisheye buffer.
struct InstaPixelLens {
    let xi: Float
    let fx: Float
    let fy: Float
    let cx: Float
    let cy: Float
    let v6: Bool
    let k0: Float, k1: Float, k2: Float, k3: Float, k4: Float
    let k5: Float, k6: Float, k7: Float, k8: Float, k9: Float
    let k10: Float, k11: Float, k12: Float
    /// Cosine of the field limit, less a small margin (insv_calib.py).
    let cosLimit: Float
    /// This lens's half of the buffer. Samples stay one pixel inside it,
    /// so bilinear sampling never reads the other lens.
    let minX: Float
    let maxX: Float
    let maxY: Float

    init(
        _ lens: InstaLensCalibration.Lens, index: Int,
        bufferWidth: Int, bufferHeight: Int
    ) {
        let lensW = Float(bufferWidth) / 2
        let lensH = Float(bufferHeight)
        let s = lensW / lens.windowW
        let sv = lensH / lens.windowH
        xi = lens.xi
        fx = lens.fx * s
        fy = lens.fy * sv
        cx = Float(index) * lensW + s * (lens.cx + 0.5) - 0.5
        cy = sv * (lens.cy + 0.5) - 0.5
        v6 = lens.d.count == 13
        let d = lens.d + [Float](repeating: 0, count: max(0, 13 - lens.d.count))
        k0 = d[0]; k1 = d[1]; k2 = d[2]; k3 = d[3]; k4 = d[4]
        k5 = d[5]; k6 = d[6]; k7 = d[7]; k8 = d[8]; k9 = d[9]
        k10 = d[10]; k11 = d[11]; k12 = d[12]
        cosLimit = cos(lens.fieldLimitDeg * .pi / 180) - 0.01
        minX = Float(index) * lensW
        maxX = Float(index + 1) * lensW - 1
        maxY = lensH - 1
    }

    /// Buffer pixel (pixel centres at integers) that sees a ray given in
    /// this lens's frame, or nil if the lens doesn't see it. The ray needn't
    /// be unit length.
    @inline(__always)
    func project(_ x: Float, _ y: Float, _ z: Float) -> SIMD2<Float>? {
        let n = (x * x + y * y + z * z).squareRoot()
        guard n > 0, z >= cosLimit * n else { return nil }
        let den = xi * n + z
        guard den > 1e-9 else { return nil }
        let mx = x / den
        let my = y / den
        let r2 = mx * mx + my * my
        let xd: Float
        let yd: Float
        if v6 {
            let radial = 1 + r2 * (k0 + r2 * (k1 + r2 * (k2
                + r2 * (k3 + r2 * k4))))
            let ta = k5 + k7 * r2
            let tb = k6 + k8 * r2
            let mxy = 2 * mx * my
            xd = mx * radial + (r2 + 2 * mx * mx) * ta + mxy * tb
                + k9 * r2 + k11 * r2 * r2
            yd = my * radial + (r2 + 2 * my * my) * tb + mxy * ta
                + k10 * r2 + k12 * r2 * r2
        } else {
            // Version 3: k1 k2 k3 p1 p2 (OpenCV order).
            let radial = 1 + r2 * (k0 + r2 * (k1 + r2 * k2))
            xd = mx * radial + 2 * k3 * mx * my + k4 * (r2 + 2 * mx * mx)
            yd = my * radial + k3 * (r2 + 2 * my * my) + 2 * k4 * mx * my
        }
        let u = cx + fx * xd
        let v = cy + fy * yd
        guard u >= minX, u < maxX, v >= 0, v < maxY else { return nil }
        return SIMD2(u, v)
    }
}
