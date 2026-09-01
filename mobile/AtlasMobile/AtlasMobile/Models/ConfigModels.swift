import Foundation
import simd

struct RigidTransform: Codable {
    let roll: Double
    let pitch: Double
    let yaw: Double
    let x: Double
    let y: Double
    let z: Double

    /// Converts RPY + XYZ to a 4x4 homogeneous transform matrix.
    func toMatrix() -> simd_float4x4 {
        let r = Float(roll * .pi / 180)
        let p = Float(pitch * .pi / 180)
        let y = Float(yaw * .pi / 180)

        let cr = cos(r); let sr = sin(r)
        let cp = cos(p); let sp = sin(p)
        let cy = cos(y); let sy = sin(y)

        let rotation = simd_float3x3(rows: [
            SIMD3(cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            SIMD3(sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            SIMD3(-sp, cp * sr, cp * cr)
        ])

        var matrix = simd_float4x4(1.0)
        matrix.columns.0 = SIMD4(rotation.columns.0, 0)
        matrix.columns.1 = SIMD4(rotation.columns.1, 0)
        matrix.columns.2 = SIMD4(rotation.columns.2, 0)
        matrix.columns.3 = SIMD4(Float(self.x), Float(self.y), Float(self.z), 1)

        return matrix
    }
}

struct CameraConfig: Codable {
    let id: String
    let model: String
    let serial: String
    let extrinsic: RigidTransform
    let mask: String?
    let faceCount: Int
    let tileFov: Double

    enum CodingKeys: String, CodingKey {
        case id, model, serial, extrinsic, mask
        case faceCount = "face_count"
        case tileFov = "tile_fov"
    }
}

struct IPhoneConfig: Codable {
    let mask: String?
}

struct MultiCameraConfig: Codable {
    let cameras: [CameraConfig]
    let iphone: IPhoneConfig

    static let `default` = MultiCameraConfig(
        cameras: [],
        iphone: IPhoneConfig(mask: nil)
    )

    static func load(from url: URL) throws -> MultiCameraConfig {
        let text = try String(contentsOf: url, encoding: .utf8)
        return try parseYAML(text)
    }

    /// Saves the config as YAML to Documents/atlas_sessions/multi_camera.yaml.
    func saveToDocuments() throws {
        guard let documentsDir = FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first else { return }
        let dir = documentsDir.appendingPathComponent("atlas_sessions")
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        let url = dir.appendingPathComponent("multi_camera.yaml")
        try toYAML().write(to: url, atomically: true, encoding: .utf8)
    }

    private func toYAML() -> String {
        var lines = ["cameras:"]
        for cam in cameras {
            let e = cam.extrinsic
            lines += [
                "  - id: \(cam.id)",
                "    model: \(cam.model)",
                "    serial: \"\(cam.serial)\"",
                "    extrinsic:",
                "      roll: \(e.roll)",
                "      pitch: \(e.pitch)",
                "      yaw: \(e.yaw)",
                "      x: \(e.x)",
                "      y: \(e.y)",
                "      z: \(e.z)",
                "    mask: \(cam.mask ?? "null")",
                "    face_count: \(cam.faceCount)",
                "    tile_fov: \(cam.tileFov)",
            ]
        }
        lines += ["iphone:", "  mask: \(iphone.mask ?? "null")"]
        return lines.joined(separator: "\n") + "\n"
    }

    /// Loads config from Documents/atlas_sessions/multi_camera.yaml (user-placed
    /// calibrated file) falling back to the app bundle's multi_camera.yaml.
    static func loadFromDeviceOrBundle() -> MultiCameraConfig {
        // 1. User-placed calibrated file in Documents (survives app updates)
        if let documentsDir = FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first {
            let url = documentsDir
                .appendingPathComponent("atlas_sessions")
                .appendingPathComponent("multi_camera.yaml")
            if let config = try? load(from: url) {
                return config
            }
        }
        // 2. App bundle (baked in at build time)
        if let bundleURL = Bundle.main.url(forResource: "multi_camera", withExtension: "yaml"),
           let config = try? load(from: bundleURL) {
            return config
        }
        return .default
    }

    /// Minimal YAML parser for the specific multi_camera.yaml structure.
    /// Handles the atlas-scanner format without requiring a third-party YAML library.
    /// Replace with Yams (SPM) if the schema grows more complex.
    static func parseYAML(_ text: String) throws -> MultiCameraConfig {
        var cameras: [CameraConfig] = []
        var iphoneMask: String? = nil

        let lines = text.components(separatedBy: .newlines)
        var i = 0

        // Returns the indent level (number of leading spaces) of a line
        func indent(_ s: String) -> Int { s.prefix(while: { $0 == " " }).count }
        // Strips leading/trailing whitespace and inline comments
        func value(of line: String) -> String {
            let stripped = line.trimmingCharacters(in: .whitespaces)
            let noComment = stripped.components(separatedBy: " #").first ?? stripped
            // Remove surrounding quotes
            return noComment
                .trimmingCharacters(in: CharacterSet(charactersIn: "\"'"))
        }
        func keyValue(_ line: String) -> (String, String)? {
            let parts = line.trimmingCharacters(in: .whitespaces).components(separatedBy: ": ")
            guard parts.count >= 2 else { return nil }
            let k = parts[0].trimmingCharacters(in: .whitespaces)
            let v = parts.dropFirst().joined(separator: ": ")
                .trimmingCharacters(in: CharacterSet(charactersIn: " \"'"))
            return (k, v)
        }

        while i < lines.count {
            let line = lines[i]
            let trimmed = line.trimmingCharacters(in: .whitespaces)

            // Top-level cameras list
            if trimmed == "cameras:" {
                i += 1
                while i < lines.count {
                    let camLine = lines[i]
                    let camTrimmed = camLine.trimmingCharacters(in: .whitespaces)
                    guard indent(camLine) >= 2 else { break }

                    if camTrimmed.hasPrefix("- ") || camTrimmed == "-" {
                        // Start of a camera block
                        var id = "", model = "", serial = "", mask: String? = nil
                        var faceCount = 8
                        var tileFov = 65.0
                        var roll = 0.0, pitch = 0.0, yaw = 0.0
                        var x = 0.0, y = 0.0, z = 0.0
                        i += 1

                        while i < lines.count {
                            let fl = lines[i]
                            let ft = fl.trimmingCharacters(in: .whitespaces)
                            // Stop when we hit the next list item or dedent to cameras level
                            if ft.hasPrefix("- ") || (indent(fl) <= 2 && !ft.isEmpty && !ft.hasPrefix("#")) { break }
                            if ft.isEmpty || ft.hasPrefix("#") { i += 1; continue }

                            if ft == "extrinsic:" {
                                i += 1
                                while i < lines.count {
                                    let el = lines[i]
                                    let et = el.trimmingCharacters(in: .whitespaces)
                                    guard indent(el) >= 6 else { break }
                                    if let (k, v) = keyValue(el) {
                                        switch k {
                                        case "roll":  roll  = Double(v) ?? 0
                                        case "pitch": pitch = Double(v) ?? 0
                                        case "yaw":   yaw   = Double(v) ?? 0
                                        case "x":     x     = Double(v) ?? 0
                                        case "y":     y     = Double(v) ?? 0
                                        case "z":     z     = Double(v) ?? 0
                                        default: break
                                        }
                                    }
                                    i += 1
                                }
                                continue
                            }

                            if let (k, v) = keyValue(fl) {
                                switch k {
                                case "id":         id         = v
                                case "model":      model      = v
                                case "serial":     serial     = v
                                case "mask":       mask       = v == "null" || v.isEmpty ? nil : v
                                case "face_count": faceCount  = Int(v) ?? 8
                                case "tile_fov":   tileFov    = Double(v) ?? 65.0
                                default: break
                                }
                            }
                            i += 1
                        }

                        if !id.isEmpty {
                            cameras.append(CameraConfig(
                                id: id, model: model, serial: serial,
                                extrinsic: RigidTransform(roll: roll, pitch: pitch, yaw: yaw, x: x, y: y, z: z),
                                mask: mask, faceCount: faceCount, tileFov: tileFov
                            ))
                        }
                    } else {
                        i += 1
                    }
                }
                continue
            }

            // Top-level iphone block
            if trimmed == "iphone:" {
                i += 1
                while i < lines.count {
                    let il = lines[i]
                    guard indent(il) >= 2 else { break }
                    if let (k, v) = keyValue(il), k == "mask" {
                        iphoneMask = v == "null" || v.isEmpty ? nil : v
                    }
                    i += 1
                }
                continue
            }

            i += 1
        }

        return MultiCameraConfig(cameras: cameras, iphone: IPhoneConfig(mask: iphoneMask))
    }
}
