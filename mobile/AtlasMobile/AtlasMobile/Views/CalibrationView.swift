import SwiftUI
import AudioToolbox
import UIKit
import CoreVideo

struct CalibrationView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager
    @StateObject private var calibManager = CalibrationManager()

    @State private var forwardM: Double = 0.0
    @State private var leftM:    Double = 0.0
    @State private var upM:      Double = 0.0
    @State private var rollDeg:  Double = 0.0
    @State private var pitchDeg: Double = 0.0
    @State private var yawDeg:   Double = 0.0

    @State private var useMetric = true
    @State private var isCapturingFrame = false
    @State private var captureError: String?
    @State private var captureInfo: String?
    @State private var lastCaptureTime: Date?
    @State private var lastSuccessfulERP: UIImage?
    @State private var iphoneThumbnail: UIImage?
    @State private var insta360Thumbnail: UIImage?
    @State private var captureCount: Int = 0

    private let inchesToM = 0.0254

    var seed: RigidTransform {
        RigidTransform(
            roll: rollDeg, pitch: pitchDeg, yaw: yawDeg,
            x: forwardM, y: leftM, z: upM
        )
    }

    private var displayForward: Binding<Double> {
        unitBinding($forwardM)
    }
    private var displayLeft: Binding<Double> {
        unitBinding($leftM)
    }
    private var displayUp: Binding<Double> {
        unitBinding($upM)
    }

    private func unitBinding(_ source: Binding<Double>) -> Binding<Double> {
        Binding(
            get: { useMetric ? source.wrappedValue : source.wrappedValue / inchesToM },
            set: { source.wrappedValue = useMetric ? $0 : $0 * inchesToM }
        )
    }

    private var unitLabel: String { useMetric ? "m" : "in" }

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                statusBanner
                seedSection
                calibrationSection
                resultsSection
                overlaySection
            }
            .padding()
        }
        .navigationTitle("Calibration")
        .onAppear { loadSeedFromConfig() }
    }

    private var seedSection: some View {
        GroupBox("Physical Seed — Mount Measurements") {
            VStack(spacing: 10) {
                seedReferenceNote
                Picker("Units", selection: $useMetric) {
                    Text("Metric (m)").tag(true)
                    Text("Imperial (in)").tag(false)
                }
                .pickerStyle(.segmented)

                SeedField("Forward / X (\(unitLabel))", value: displayForward)
                SeedField("Left / Y (\(unitLabel))",    value: displayLeft)
                SeedField("Up / Z (\(unitLabel))",      value: displayUp)
                Divider()
                SeedField("Roll  (° about X)",  value: $rollDeg)
                SeedField("Pitch (° about Y)",  value: $pitchDeg)
                SeedField("Yaw   (° about Z)",  value: $yawDeg)
            }
            .padding(.top, 4)
        }
    }

    private var seedReferenceNote: some View {
        let note = "iPhone → Insta360 offset. Hold phone "
            + "upright with screen facing you:\n"
            + "• Forward = away from you (camera direction)\n"
            + "• Left = your left\n"
            + "• Up = toward sky\n"
            + "Right-hand Z-up, rotations in ZYX order."
        return Text(note)
            .font(.caption2)
            .foregroundColor(.secondary)
            .frame(maxWidth: .infinity, alignment: .leading)
    }

    private var calibrationSection: some View {
        GroupBox("Calibration") {
            VStack(spacing: 12) {
                Button {
                    Task { await captureFrame() }
                } label: {
                    Label(
                        isCapturingFrame
                            ? "Capturing…"
                            : "Capture Frame (\(frameCount))",
                        systemImage: "camera.fill"
                    )
                }
                .buttonStyle(.borderedProminent)
                .disabled(
                    !sessionManager.isSessionActive || isCapturingFrame
                )

                if let error = captureError {
                    Label(error, systemImage: "exclamationmark.triangle")
                        .font(.caption)
                        .foregroundColor(.red)
                }

                if let info = captureInfo {
                    Text(info)
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundColor(.secondary)
                        .lineLimit(4)
                }

                if iphoneThumbnail != nil || insta360Thumbnail != nil {
                    HStack(spacing: 8) {
                        if let thumb = iphoneThumbnail {
                            VStack(spacing: 2) {
                                Image(uiImage: thumb)
                                    .resizable()
                                    .scaledToFill()
                                    .frame(width: 150, height: 100)
                                    .clipped()
                                    .cornerRadius(6)
                                    .id("iphone-\(captureCount)")
                                Text("iPhone")
                                    .font(.caption2)
                                    .foregroundColor(.secondary)
                            }
                        }
                        if let erp = insta360Thumbnail {
                            VStack(spacing: 2) {
                                Image(uiImage: erp)
                                    .resizable()
                                    .scaledToFill()
                                    .frame(width: 150, height: 100)
                                    .clipped()
                                    .cornerRadius(6)
                                    .id("insta-\(captureCount)")
                                Text("Insta360")
                                    .font(.caption2)
                                    .foregroundColor(.secondary)
                            }
                        }
                    }
                }

                Button(action: reRenderOverlay) {
                    Label(
                        "Re-render Overlay",
                        systemImage: "arrow.clockwise"
                    )
                }
                .buttonStyle(.bordered)
                .disabled(frameCount == 0)

                Button(action: runOptimization) {
                    Label("Run Optimisation", systemImage: "wand.and.stars")
                }
                .buttonStyle(.bordered)
                .disabled(frameCount == 0 || isOptimizing)

                if isOptimizing {
                    ProgressView("Optimising…")
                }

                if let optError = calibManager.optimizeError {
                    Label(optError, systemImage: "exclamationmark.triangle")
                        .font(.caption)
                        .foregroundColor(.orange)
                }

                if calibManager.matchCount > 0 {
                    Label(
                        "\(calibManager.matchCount) keypoint matches",
                        systemImage: "point.3.connected.trianglepath.dotted"
                    )
                    .font(.caption)
                    .foregroundColor(.secondary)
                }

                Button(action: { calibManager.clearFrames() }) {
                    Label("Clear Frames", systemImage: "trash")
                }
                .buttonStyle(.bordered)
                .foregroundColor(.red)
                .disabled(frameCount == 0)
            }
            .frame(maxWidth: .infinity)
            .padding(.top, 4)
        }
    }

    @ViewBuilder
    private var resultsSection: some View {
        if case .done(let result) = calibManager.state {
            GroupBox("Refined Extrinsic") {
                VStack(alignment: .leading, spacing: 6) {
                    resultText(result)
                    Button("Save to Device") {
                        saveCalibration(result)
                    }
                    .buttonStyle(.borderedProminent)
                    .padding(.top, 4)
                }
                .padding(.top, 4)
            }
        }
        if case .saved = calibManager.state {
            Label(
                "Saved to Documents/atlas_sessions/multi_camera.yaml",
                systemImage: "checkmark.circle.fill"
            )
            .foregroundColor(.green)
            .font(.caption)
        }
    }

    private func resultText(_ r: RigidTransform) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("RPY: \(r.roll, specifier: "%.3f")°  \(r.pitch, specifier: "%.3f")°  \(r.yaw, specifier: "%.3f")°")
                .font(.system(.body, design: .monospaced))
            Text("XYZ: \(r.x, specifier: "%.4f")  \(r.y, specifier: "%.4f")  \(r.z, specifier: "%.4f") m")
                .font(.system(.body, design: .monospaced))
            Text("Cost: \(calibManager.reprojectionError, specifier: "%.4f")")
                .font(.caption)
                .foregroundColor(.secondary)
        }
    }

    @ViewBuilder
    private var overlaySection: some View {
        if let preview = calibManager.overlayImage {
            GroupBox("Verification Overlay") {
                VStack(alignment: .leading, spacing: 6) {
                    Image(uiImage: preview)
                        .resizable()
                        .scaledToFit()
                        .cornerRadius(8)
                        .id(calibManager.overlayVersion)
                    Text("Overlay #\(calibManager.overlayVersion)")
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundColor(.secondary)
                    Text("Left: edges (red=camera green=LiDAR yellow=aligned)")
                        .font(.caption2).foregroundColor(.secondary)
                    Text("Right: TURBO LiDAR dots (near=warm far=cool)")
                        .font(.caption2).foregroundColor(.secondary)
                    Text("Shifted horizontally → adjust Yaw")
                        .font(.caption2)
                    Text("Shifted vertically → adjust Pitch")
                        .font(.caption2)
                    Text("Rotated → adjust Roll")
                        .font(.caption2)
                    if !calibManager.frameDebugInfo.isEmpty {
                        Text(calibManager.frameDebugInfo)
                            .font(.system(.caption2, design: .monospaced))
                            .foregroundColor(.secondary)
                    }
                }
                .padding(.top, 4)
            }
        }

        Text("Build: 2026-10-03f")
            .font(.system(.caption2, design: .monospaced))
            .foregroundColor(.secondary.opacity(0.5))
    }

    // MARK: - Computed

    private var frameCount: Int {
        if case .framesCollected(let n) = calibManager.state { return n }
        if case .done = calibManager.state { return -1 }
        return 0
    }

    private var isOptimizing: Bool {
        if case .optimizing = calibManager.state { return true }
        return false
    }

    private var statusBanner: some View {
        HStack {
            Circle()
                .fill(statusColor)
                .frame(width: 10, height: 10)
            Text(statusText)
                .font(.subheadline)
        }
        .padding(10)
        .background(.ultraThinMaterial)
        .cornerRadius(8)
    }

    private var statusColor: Color {
        switch calibManager.state {
        case .idle:               return .gray
        case .framesCollected:    return .orange
        case .optimizing:         return .blue
        case .done:               return .yellow
        case .saved:              return .green
        }
    }

    private var statusText: String {
        switch calibManager.state {
        case .idle:
            return sessionManager.isSessionActive
                ? "Press Capture Frame to begin"
                : "Start a session first, then return here"
        case .framesCollected(let n):  return "\(n) frame\(n == 1 ? "" : "s") captured"
        case .optimizing:              return "Optimising…"
        case .done:                    return "Optimisation complete — review overlay"
        case .saved:                   return "Calibration saved"
        }
    }

    // MARK: - Actions

    private func loadSeedFromConfig() {
        let config = MultiCameraConfig.loadFromDeviceOrBundle()
        guard let cam = config.cameras.first else { return }
        forwardM = cam.extrinsic.x
        leftM    = cam.extrinsic.y
        upM      = cam.extrinsic.z
        rollDeg  = cam.extrinsic.roll
        pitchDeg = cam.extrinsic.pitch
        yawDeg   = cam.extrinsic.yaw
    }

    private static let captureCooldownSeconds: TimeInterval = 4

    private func captureFrame() async {
        captureError = nil

        if let last = lastCaptureTime,
           Date().timeIntervalSince(last) < Self.captureCooldownSeconds {
            captureError = "Wait a moment between captures"
            return
        }

        isCapturingFrame = true
        defer {
            isCapturingFrame = false
            lastCaptureTime = Date()
        }

        // ── Phase 1: iPhone capture (instant) ──────────────────────

        guard let arkitFrame =
            sessionManager.arkitCapture.captureCurrentFrame()
        else {
            captureError = "No ARKit frame — is the session active?"
            return
        }

        iphoneThumbnail = sessionManager.thumbnailFromPixelBuffer(
            arkitFrame.capturedImage
        )

        // Copy depth NOW — ARKit recycles CVPixelBuffers during awaits.
        let depthBuf = arkitFrame.depthMap
        let depthW = CVPixelBufferGetWidth(depthBuf)
        let depthH = CVPixelBufferGetHeight(depthBuf)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthBuf)
        CVPixelBufferLockBaseAddress(depthBuf, .readOnly)
        let depthFloats: [Float]
        if let base = CVPixelBufferGetBaseAddress(depthBuf) {
            let floatsPerRow = bytesPerRow / MemoryLayout<Float>.size
            if floatsPerRow == depthW {
                depthFloats = Array(UnsafeBufferPointer(
                    start: base.bindMemory(
                        to: Float.self, capacity: depthW * depthH
                    ),
                    count: depthW * depthH
                ))
            } else {
                var arr = [Float](
                    repeating: 0, count: depthW * depthH
                )
                for row in 0..<depthH {
                    let rowBase = base.advanced(by: row * bytesPerRow)
                        .assumingMemoryBound(to: Float.self)
                    for col in 0..<depthW {
                        arr[row * depthW + col] = rowBase[col]
                    }
                }
                depthFloats = arr
            }
        } else {
            CVPixelBufferUnlockBaseAddress(depthBuf, .readOnly)
            captureError = "Could not read depth buffer"
            return
        }
        CVPixelBufferUnlockBaseAddress(depthBuf, .readOnly)

        // Immediate feedback — iPhone has captured.
        sessionManager.captureFlash = true
        AudioServicesPlaySystemSound(1108)
        UIImpactFeedbackGenerator(style: .medium).impactOccurred()

        // Dimensions for intrinsics: buffer dims are always landscape.
        let bufW = CVPixelBufferGetWidth(arkitFrame.capturedImage)
        let bufH = CVPixelBufferGetHeight(arkitFrame.capturedImage)
        let resW = Int(arkitFrame.imageResolution.width)
        let resH = Int(arkitFrame.imageResolution.height)

        print("[Calib] depth=\(depthW)x\(depthH) bpr=\(bytesPerRow)"
            + " buf=\(bufW)x\(bufH) res=\(resW)x\(resH)"
            + " fx=\(arkitFrame.intrinsics[0][0])"
            + " fy=\(arkitFrame.intrinsics[1][1])"
            + " cx=\(arkitFrame.intrinsics[2][0])"
            + " cy=\(arkitFrame.intrinsics[2][1])"
            + " portrait=\(resW < resH)")

        // Let SwiftUI show flash + thumbnail before Insta360 blocks.
        await Task.yield()

        // ── Phase 2: Insta360 capture (WiFi, 1-3 s) ───────────────

        let captured = await sessionManager.triggerInsta360Capture()

        // Download and stitch the Insta360 image; fall back to the
        // previous successful ERP so the overlay always renders.
        var instaERP: UIImage?
        var downloadWarning: String?

        if !captured {
            downloadWarning = "No 360° capture — using previous ERP"
        } else if let fullImage =
            await sessionManager.downloadLastInsta360Image() {
            if DualFisheyeStitcher.isDualFisheye(fullImage) {
                let src = fullImage
                let stitched = await Task.detached(
                    priority: .userInitiated
                ) {
                    DualFisheyeStitcher.stitch(src, outputWidth: 2048)
                }.value
                instaERP = stitched
            } else {
                instaERP = Self.downsampleERP(fullImage, maxWidth: 2048)
            }
        } else {
            downloadWarning = "Download failed — using previous ERP"
            print("[Calib] download failed, falling back to previous ERP")
        }

        if let erp = instaERP {
            lastSuccessfulERP = erp
            insta360Thumbnail = erp
        } else {
            instaERP = lastSuccessfulERP
        }
        captureCount += 1

        guard let erpImage = instaERP else {
            captureError = "No 360° image available"
            return
        }

        if let warning = downloadWarning {
            captureError = warning
        }

        // ── Phase 3: Build frame + render overlay ──────────────────

        let frame = CalibrationFrame(
            depth: depthFloats,
            depthW: depthW, depthH: depthH,
            intrinsics: arkitFrame.intrinsics,
            imageW: bufW, imageH: bufH,
            imageResW: resW, imageResH: resH,
            erpImage: erpImage
        )
        captureInfo = frame.debugDescription
        print("[Calib] \(frame.debugDescription)")

        calibManager.addFrame(frame)

        let overlay = CalibrationOverlayRenderer.render(
            frame: frame, extrinsic: seed
        )
        print("[Calib] overlay render: \(overlay != nil ? "OK" : "nil")")
        calibManager.overlayImage = overlay
        calibManager.overlayVersion += 1
    }

    private func reRenderOverlay() {
        guard let frame = calibManager.lastFrame else { return }
        calibManager.overlayImage =
            CalibrationOverlayRenderer.render(
                frame: frame, extrinsic: seed
            )
        calibManager.overlayVersion += 1
    }

    private func runOptimization() {
        Task { await calibManager.optimize(seed: seed) }
    }

    private func saveCalibration(_ result: RigidTransform) {
        try? calibManager.save(refined: result)
    }

    private static func downsampleERP(
        _ image: UIImage, maxWidth: Int
    ) -> UIImage {
        let w = Int(image.size.width)
        guard w > maxWidth else { return image }
        let scale = CGFloat(maxWidth) / CGFloat(w)
        let newSize = CGSize(
            width: CGFloat(maxWidth),
            height: image.size.height * scale
        )
        let renderer = UIGraphicsImageRenderer(size: newSize)
        return renderer.image { _ in
            image.draw(in: CGRect(origin: .zero, size: newSize))
        }
    }
}

// MARK: - Seed input field

private struct SeedField: View {
    let label: String
    @Binding var value: Double

    init(_ label: String, value: Binding<Double>) {
        self.label = label
        self._value = value
    }

    var body: some View {
        HStack {
            Text(label)
                .frame(width: 110, alignment: .leading)
                .font(.caption)
            Spacer()
            HStack(spacing: 4) {
                Button {
                    value = -value
                } label: {
                    Text("±")
                        .font(.body.weight(.semibold))
                        .frame(width: 30, height: 30)
                        .background(Color(.systemGray5))
                        .cornerRadius(6)
                }
                .buttonStyle(.plain)
                TextField("0.0", value: $value, format: .number)
                    .keyboardType(.decimalPad)
                    .multilineTextAlignment(.trailing)
                    .frame(width: 80)
                    .textFieldStyle(.roundedBorder)
            }
        }
    }
}
