import SwiftUI

struct CalibrationView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager
    @StateObject private var calibManager = CalibrationManager()

    // Physical seed inputs (inches for translation, degrees for rotation)
    @State private var forwardIn: Double = 0.0
    @State private var leftIn:    Double = 0.0
    @State private var upIn:      Double = 0.0
    @State private var rollDeg:   Double = 0.0
    @State private var pitchDeg:  Double = 0.0
    @State private var yawDeg:    Double = 0.0

    @State private var showSaveConfirmation = false

    private let inchesToM = 0.0254

    var seed: RigidTransform {
        RigidTransform(
            roll: rollDeg, pitch: pitchDeg, yaw: yawDeg,
            x: forwardIn * inchesToM,
            y: leftIn    * inchesToM,
            z: upIn      * inchesToM
        )
    }

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {

                // Status
                statusBanner

                // Seed inputs
                GroupBox("Physical Seed — Mount Measurements") {
                    VStack(spacing: 10) {
                        SeedField("Forward (in)", value: $forwardIn)
                        SeedField("Left (in)",    value: $leftIn)
                        SeedField("Up (in)",      value: $upIn)
                        Divider()
                        SeedField("Roll (°)",     value: $rollDeg)
                        SeedField("Pitch (°)",    value: $pitchDeg)
                        SeedField("Yaw (°)",      value: $yawDeg)
                    }
                    .padding(.top, 4)
                }

                // Actions
                GroupBox("Calibration") {
                    VStack(spacing: 12) {
                        Button(action: captureFrame) {
                            Label("Capture Frame (\(frameCount))", systemImage: "camera.fill")
                        }
                        .buttonStyle(.borderedProminent)
                        .disabled(!sessionManager.isSessionActive)

                        Button(action: runOptimization) {
                            Label("Run Optimisation", systemImage: "wand.and.stars")
                        }
                        .buttonStyle(.bordered)
                        .disabled(frameCount == 0 || isOptimizing)

                        if isOptimizing {
                            ProgressView("Optimising…")
                        }

                        if calibManager.matchCount > 0 {
                            Label("\(calibManager.matchCount) keypoint matches",
                                  systemImage: "point.3.connected.trianglepath.dotted")
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

                // Results
                if case .done(let result) = calibManager.state {
                    GroupBox("Refined Extrinsic") {
                        VStack(alignment: .leading, spacing: 6) {
                            Text("RPY: \(result.roll, specifier: "%.3f")°  \(result.pitch, specifier: "%.3f")°  \(result.yaw, specifier: "%.3f")°")
                                .font(.system(.body, design: .monospaced))
                            Text("XYZ: \(result.x, specifier: "%.4f")  \(result.y, specifier: "%.4f")  \(result.z, specifier: "%.4f") m")
                                .font(.system(.body, design: .monospaced))
                            Text("Cost: \(calibManager.reprojectionError, specifier: "%.4f")")
                                .font(.caption)
                                .foregroundColor(.secondary)

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
                    Label("Saved to Documents/atlas_sessions/multi_camera.yaml",
                          systemImage: "checkmark.circle.fill")
                        .foregroundColor(.green)
                        .font(.caption)
                }

                // Overlay image
                if let overlay = calibManager.overlayImage {
                    GroupBox("Verification Overlay") {
                        VStack(alignment: .leading, spacing: 6) {
                            Image(uiImage: overlay)
                                .resizable()
                                .scaledToFit()
                                .cornerRadius(8)
                            Text("Left: red=Insta360 edges  green=LiDAR  yellow=aligned")
                                .font(.caption2)
                                .foregroundColor(.secondary)
                            Text("Right: LiDAR depth dots on panorama (near=white, far=blue)")
                                .font(.caption2)
                                .foregroundColor(.secondary)
                            Text("Shifted horizontally → adjust Yaw")
                                .font(.caption2)
                            Text("Shifted vertically → adjust Pitch")
                                .font(.caption2)
                            Text("Rotated → adjust Roll")
                                .font(.caption2)
                        }
                        .padding(.top, 4)
                    }
                }
            }
            .padding()
        }
        .navigationTitle("Calibration")
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
        case .idle:                    return "No frames — start a session and capture"
        case .framesCollected(let n):  return "\(n) frame\(n == 1 ? "" : "s") captured"
        case .optimizing:              return "Optimising…"
        case .done:                    return "Optimisation complete — review overlay"
        case .saved:                   return "Calibration saved"
        }
    }

    // MARK: - Actions

    private func captureFrame() {
        guard let arkitFrame = sessionManager.arkitCapture.captureCurrentFrame() else { return }
        guard let instaERP = sessionManager.lastInstaERP else { return }

        guard let depthMap = arkitFrame.depthMap as CVPixelBuffer? else { return }
        let w = CVPixelBufferGetWidth(depthMap)
        let h = CVPixelBufferGetHeight(depthMap)
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(depthMap) else { return }
        let floats = Array(UnsafeBufferPointer(
            start: base.bindMemory(to: Float.self, capacity: w * h),
            count: w * h
        ))

        let frame = CalibrationFrame(
            depth: floats,
            depthW: w, depthH: h,
            intrinsics: arkitFrame.intrinsics,
            imageW: Int(arkitFrame.imageResolution.width),
            imageH: Int(arkitFrame.imageResolution.height),
            erpImage: instaERP
        )
        calibManager.addFrame(frame)
    }

    private func runOptimization() {
        Task { await calibManager.optimize(seed: seed) }
    }

    private func saveCalibration(_ result: RigidTransform) {
        try? calibManager.save(refined: result)
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
            TextField("0.0", value: $value, format: .number)
                .keyboardType(.decimalPad)
                .multilineTextAlignment(.trailing)
                .frame(width: 80)
                .textFieldStyle(.roundedBorder)
        }
    }
}
