import Foundation
import CoreML
import UIKit

// MARK: - Export mode

enum ExportMode: String, CaseIterable, Identifiable {
    case colmapOnly    = "COLMAP Only"
    case fullOnDevice  = "Full On-Device"
    case hostProcessing = "Host Processing"

    var id: String { rawValue }

    var description: String {
        switch self {
        case .colmapOnly:
            return "Export COLMAP model files only. Fast, no ML required. " +
                   "Run enhance_session.py on a host for dense depth + normals."
        case .fullOnDevice:
            return "COLMAP + PromptDA dense depth + StableNormal surface normals. " +
                   "Runs entirely on-device via CoreML. ~1–3s/scan depth, ~30s/scan normals."
        case .hostProcessing:
            return "Upload session to a host running enhance_server.py. " +
                   "Host runs PromptDA + StableNormal on GPU and returns enhanced results."
        }
    }

    var systemImage: String {
        switch self {
        case .colmapOnly:    return "cube.transparent"
        case .fullOnDevice:  return "iphone.radiowaves.left.and.right"
        case .hostProcessing: return "server.rack"
        }
    }
}

// MARK: - Progress

struct ExportProgress {
    var stage: String = ""
    var current: Int = 0
    var total: Int = 0
    var isComplete: Bool = false
    var error: String? = nil

    var fraction: Double {
        total > 0 ? Double(current) / Double(total) : 0
    }
}

// MARK: - On-device post-processing

/// Runs PromptDA and StableNormal CoreML models on-device.
/// Models must be bundled as PromptDA.mlpackage and StableNormal.mlpackage.
@MainActor
final class PostProcessingManager: ObservableObject {
    @Published var progress = ExportProgress()

    private var promptDAModel: MLModel?
    private var stableNormalModel: MLModel?

    // MARK: - Model loading

    func loadModels() async {
        await Task.detached(priority: .userInitiated) {
            // PromptDA
            if let url = Bundle.main.url(forResource: "PromptDA", withExtension: "mlpackage") {
                self.promptDAModel = try? MLModel(contentsOf: url)
            }
            // StableNormal
            if let url = Bundle.main.url(forResource: "StableNormal", withExtension: "mlpackage") {
                self.stableNormalModel = try? MLModel(contentsOf: url)
            }
        }.value
    }

    var promptDAAvailable: Bool { promptDAModel != nil }
    var stableNormalAvailable: Bool { stableNormalModel != nil }

    // MARK: - Full pipeline

    func runFullPipeline(sessionDirectory: URL) async {
        let rawDir = sessionDirectory.appendingPathComponent("raw/iphone")
        guard let scanDirs = try? FileManager.default
            .contentsOfDirectory(at: rawDir, includingPropertiesForKeys: nil)
            .filter({ $0.lastPathComponent.hasPrefix("scan_") })
            .sorted(by: { $0.path < $1.path })
        else { return }

        let total = scanDirs.count
        progress = ExportProgress(stage: "PromptDA depth completion", total: total * 2)

        // Stage 1: PromptDA
        if promptDAAvailable {
            for (i, scanDir) in scanDirs.enumerated() {
                progress.stage = "PromptDA: \(scanDir.lastPathComponent)"
                progress.current = i
                await runPromptDA(scanDir: scanDir, sessionDirectory: sessionDirectory)
            }
        }

        // Stage 2: StableNormal
        if stableNormalAvailable {
            for (i, scanDir) in scanDirs.enumerated() {
                progress.stage = "StableNormal: \(scanDir.lastPathComponent)"
                progress.current = total + i
                await runStableNormal(scanDir: scanDir, sessionDirectory: sessionDirectory)
            }
        }

        progress = ExportProgress(stage: "Complete", current: total * 2,
                                  total: total * 2, isComplete: true)
    }

    // MARK: - PromptDA

    private func runPromptDA(scanDir: URL, sessionDirectory: URL) async {
        guard let model = promptDAModel else { return }
        let rgbURL   = scanDir.appendingPathComponent("rgb.jpg")
        let depthURL = scanDir.appendingPathComponent("depth.bin")
        let outDir   = sessionDirectory.appendingPathComponent("enhanced/depth_dense")
        let outURL   = outDir.appendingPathComponent("\(scanDir.lastPathComponent).png")

        guard FileManager.default.fileExists(atPath: rgbURL.path),
              FileManager.default.fileExists(atPath: depthURL.path),
              !FileManager.default.fileExists(atPath: outURL.path) else { return }

        try? FileManager.default.createDirectory(at: outDir, withIntermediateDirectories: true)

        await Task.detached(priority: .userInitiated) {
            guard let rgb = UIImage(contentsOfFile: rgbURL.path),
                  let rgbCG = rgb.cgImage else { return }

            // Load sparse depth (256×192 Float32)
            guard let depthData = try? Data(contentsOf: depthURL) else { return }
            let depthFloats = depthData.withUnsafeBytes {
                Array($0.bindMemory(to: Float.self).prefix(256 * 192))
            }

            // Build CoreML input
            guard let rgbBuffer = Self.cgImageToMLMultiArray(rgbCG, targetW: 518, targetH: 518),
                  let depthBuffer = Self.depthToMLMultiArray(depthFloats, w: 256, h: 192,
                                                              targetW: 518, targetH: 518)
            else { return }

            let input = try? MLDictionaryFeatureProvider(dictionary: [
                "image": MLFeatureValue(multiArray: rgbBuffer),
                "prompt_depth": MLFeatureValue(multiArray: depthBuffer),
            ])
            guard let input, let output = try? model.prediction(from: input) else { return }

            // Extract depth output and save as uint16 PNG (mm)
            guard let depthOut = output.featureValue(for: "depth")?.multiArrayValue else { return }
            let outH = Int(rgb.size.height), outW = Int(rgb.size.width)
            var depthMM = [UInt16](repeating: 0, count: outW * outH)
            let stride = depthOut.strides[0].intValue
            for i in 0..<min(outW * outH, depthOut.count) {
                let metres = Float(truncating: depthOut[i])
                depthMM[i] = UInt16(clamping: Int(metres * 1000))
            }
            Self.saveUInt16PNG(pixels: depthMM, width: outW, height: outH, to: outURL)
        }.value
    }

    // MARK: - StableNormal

    private func runStableNormal(scanDir: URL, sessionDirectory: URL) async {
        guard let model = stableNormalModel else { return }
        let rgbURL = scanDir.appendingPathComponent("rgb.jpg")
        let outDir = sessionDirectory.appendingPathComponent("enhanced/normals")
        let outURL = outDir.appendingPathComponent("\(scanDir.lastPathComponent).png")

        guard FileManager.default.fileExists(atPath: rgbURL.path),
              !FileManager.default.fileExists(atPath: outURL.path) else { return }

        try? FileManager.default.createDirectory(at: outDir, withIntermediateDirectories: true)

        await Task.detached(priority: .userInitiated) {
            guard let rgb = UIImage(contentsOfFile: rgbURL.path),
                  let rgbCG = rgb.cgImage else { return }

            guard let rgbBuffer = Self.cgImageToMLMultiArray(rgbCG, targetW: 768, targetH: 768)
            else { return }

            let input = try? MLDictionaryFeatureProvider(dictionary: [
                "image": MLFeatureValue(multiArray: rgbBuffer),
            ])
            guard let input, let output = try? model.prediction(from: input) else { return }
            guard let normalOut = output.featureValue(for: "normal")?.multiArrayValue else { return }

            // normal is (3, H, W) float32 in [-1, 1] → encode to RGB uint8
            let outH = Int(rgb.size.height), outW = Int(rgb.size.width)
            var rgba = [UInt8](repeating: 255, count: outW * outH * 4)
            let n = outW * outH
            for i in 0..<n {
                let r = Float(truncating: normalOut[i])
                let g = Float(truncating: normalOut[n + i])
                let b = Float(truncating: normalOut[2 * n + i])
                rgba[i*4]   = UInt8(clamping: Int((r * 0.5 + 0.5) * 255))
                rgba[i*4+1] = UInt8(clamping: Int((g * 0.5 + 0.5) * 255))
                rgba[i*4+2] = UInt8(clamping: Int((b * 0.5 + 0.5) * 255))
            }
            Self.saveRGBAPNG(pixels: rgba, width: outW, height: outH, to: outURL)
        }.value
    }

    // MARK: - CoreML buffer helpers

    private static func cgImageToMLMultiArray(
        _ cgImage: CGImage, targetW: Int, targetH: Int
    ) -> MLMultiArray? {
        guard let ctx = CGContext(
            data: nil, width: targetW, height: targetH,
            bitsPerComponent: 8, bytesPerRow: targetW * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return nil }
        ctx.draw(cgImage, in: CGRect(x: 0, y: 0, width: targetW, height: targetH))
        guard let ptr = ctx.data else { return nil }
        let bytes = ptr.bindMemory(to: UInt8.self, capacity: targetW * targetH * 4)

        guard let arr = try? MLMultiArray(shape: [1, 3, targetH as NSNumber, targetW as NSNumber],
                                          dataType: .float32) else { return nil }
        let n = targetW * targetH
        for i in 0..<n {
            arr[i]         = NSNumber(value: Float(bytes[i*4])   / 255.0)
            arr[n + i]     = NSNumber(value: Float(bytes[i*4+1]) / 255.0)
            arr[2 * n + i] = NSNumber(value: Float(bytes[i*4+2]) / 255.0)
        }
        return arr
    }

    private static func depthToMLMultiArray(
        _ depth: [Float], w: Int, h: Int, targetW: Int, targetH: Int
    ) -> MLMultiArray? {
        guard let arr = try? MLMultiArray(shape: [1, 1, targetH as NSNumber, targetW as NSNumber],
                                          dataType: .float32) else { return nil }
        for row in 0..<targetH {
            for col in 0..<targetW {
                let srcRow = row * h / targetH
                let srcCol = col * w / targetW
                let val = depth[min(srcRow * w + srcCol, depth.count - 1)]
                arr[row * targetW + col] = NSNumber(value: val)
            }
        }
        return arr
    }

    private static func saveUInt16PNG(pixels: [UInt16], width: Int, height: Int, to url: URL) {
        var data = pixels
        let provider = CGDataProvider(data: Data(bytes: &data,
                                                  count: width * height * 2) as CFData)!
        guard let cg = CGImage(width: width, height: height,
                               bitsPerComponent: 16, bitsPerPixel: 16,
                               bytesPerRow: width * 2,
                               space: CGColorSpaceCreateDeviceGray(),
                               bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.none.rawValue),
                               provider: provider,
                               decode: nil, shouldInterpolate: false,
                               intent: .defaultIntent) else { return }
        if let dest = CGImageDestinationCreateWithURL(url as CFURL, "public.png" as CFString, 1, nil) {
            CGImageDestinationAddImage(dest, cg, nil)
            CGImageDestinationFinalize(dest)
        }
    }

    private static func saveRGBAPNG(pixels: [UInt8], width: Int, height: Int, to url: URL) {
        var data = pixels
        let provider = CGDataProvider(data: Data(bytes: &data,
                                                  count: width * height * 4) as CFData)!
        guard let cg = CGImage(width: width, height: height,
                               bitsPerComponent: 8, bitsPerPixel: 32,
                               bytesPerRow: width * 4,
                               space: CGColorSpaceCreateDeviceRGB(),
                               bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue),
                               provider: provider,
                               decode: nil, shouldInterpolate: false,
                               intent: .defaultIntent) else { return }
        if let dest = CGImageDestinationCreateWithURL(url as CFURL, "public.png" as CFString, 1, nil) {
            CGImageDestinationAddImage(dest, cg, nil)
            CGImageDestinationFinalize(dest)
        }
    }
}
