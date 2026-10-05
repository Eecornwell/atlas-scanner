import SwiftUI
import ARKit

struct CaptureOverlayView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager
    @State private var showFlash = false
    @State private var showCalibration = false
    @State private var showConnectionLog = false

    var body: some View {
        ZStack {
            // Depth overlay (tap anywhere to dismiss)
            if sessionManager.showDepthOverlay,
               let depthImage = sessionManager.depthOverlayImage {
                Image(uiImage: depthImage)
                    .resizable()
                    .scaledToFill()
                    .ignoresSafeArea()
                    .opacity(0.6)
                    .onTapGesture { sessionManager.toggleDepthOverlay() }

                VStack {
                    Button {
                        sessionManager.toggleDepthOverlay()
                    } label: {
                        Label("Back to Camera", systemImage: "xmark.circle.fill")
                            .font(.subheadline.weight(.semibold))
                            .foregroundColor(.white)
                            .padding(.horizontal, 16)
                            .padding(.vertical, 10)
                            .background(.black.opacity(0.7))
                            .cornerRadius(20)
                    }
                    .padding(.top, 60)
                    Spacer()
                }
            }

            // Capture flash
            if showFlash {
                Color.white
                    .ignoresSafeArea()
                    .allowsHitTesting(false)
                    .transition(.opacity)
            }

            VStack {
                VStack(spacing: 6) {
                    HStack {
                        TrackingStateBadge(
                            state: sessionManager.trackingState
                        )

                        Spacer()

                        Text("Scans: \(sessionManager.scanCount)")
                            .font(.system(.headline, design: .monospaced))
                            .padding(.horizontal, 12)
                            .padding(.vertical, 6)
                            .background(.ultraThinMaterial)
                            .cornerRadius(8)
                    }

                    if let status = sessionManager.cameraStatus {
                        HStack {
                            Label(status, systemImage: cameraIcon)
                                .font(.caption.weight(.medium))
                                .foregroundColor(cameraStatusColor)
                                .padding(.horizontal, 10)
                                .padding(.vertical, 6)
                                .background(.ultraThinMaterial)
                                .cornerRadius(8)

                            if showRetryButton {
                                Button {
                                    sessionManager.retryInsta360Connection()
                                } label: {
                                    Label("Retry", systemImage: "arrow.clockwise")
                                        .font(.caption.weight(.medium))
                                        .padding(.horizontal, 10)
                                        .padding(.vertical, 6)
                                        .background(.ultraThinMaterial)
                                        .cornerRadius(8)
                                }
                            }

                            Spacer()

                            if !sessionManager.connectionLog.isEmpty {
                                Button {
                                    showConnectionLog.toggle()
                                } label: {
                                    Image(systemName: "doc.text.magnifyingglass")
                                        .font(.caption)
                                        .foregroundColor(.white)
                                        .padding(6)
                                        .background(.ultraThinMaterial)
                                        .clipShape(Circle())
                                }
                            }
                        }
                    }

                    if showConnectionLog {
                        VStack(spacing: 4) {
                            HStack {
                                Text("Connection Log")
                                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                                    .foregroundColor(.white.opacity(0.7))
                                Spacer()
                                Button {
                                    UIPasteboard.general.string = sessionManager
                                        .connectionLog.joined(separator: "\n")
                                } label: {
                                    Label("Copy", systemImage: "doc.on.doc")
                                        .font(.system(size: 10, weight: .medium))
                                        .foregroundColor(.white)
                                        .padding(.horizontal, 8)
                                        .padding(.vertical, 4)
                                        .background(.white.opacity(0.2))
                                        .cornerRadius(4)
                                }
                            }
                            ScrollView {
                                VStack(alignment: .leading, spacing: 2) {
                                    ForEach(
                                        Array(sessionManager.connectionLog.suffix(50).enumerated()),
                                        id: \.offset
                                    ) { _, line in
                                        Text(line)
                                            .font(.system(size: 10, design: .monospaced))
                                            .foregroundColor(.white)
                                    }
                                }
                                .frame(maxWidth: .infinity, alignment: .leading)
                            }
                        }
                        .frame(maxHeight: 200)
                        .padding(8)
                        .background(.black.opacity(0.7))
                        .cornerRadius(8)
                    }
                }
                .padding(.horizontal)
                .padding(.top, 8)

                Spacer()

                // Bottom: thumbnails + capture button + controls
                HStack(alignment: .bottom) {
                    // Thumbnails
                    VStack(spacing: 6) {
                        if let thumb = sessionManager.lastCapturedThumbnail {
                            ThumbnailView(image: thumb, label: "iPhone")
                                .id("iphone-\(sessionManager.thumbnailVersion)")
                        }
                        if let erp = sessionManager.lastInstaERP {
                            ThumbnailView(image: erp, label: "360")
                                .id("insta-\(sessionManager.thumbnailVersion)")
                        }
                        if sessionManager.lastCapturedThumbnail == nil
                            && sessionManager.lastInstaERP == nil {
                            Color.clear.frame(width: 80, height: 80)
                        }
                    }

                    Spacer()

                    // Capture button
                    if sessionManager.isAutoCapturing {
                        Button {
                            sessionManager.stopAutoCapture()
                        } label: {
                            ZStack {
                                RoundedRectangle(cornerRadius: 8)
                                    .fill(.red)
                                    .frame(width: 40, height: 40)
                                Circle()
                                    .stroke(.white, lineWidth: 4)
                                    .frame(width: 82, height: 82)
                            }
                        }
                    } else {
                        Button {
                            Task { await sessionManager.captureScan() }
                        } label: {
                            ZStack {
                                Circle()
                                    .fill(.white)
                                    .frame(width: 72, height: 72)
                                Circle()
                                    .stroke(.white, lineWidth: 4)
                                    .frame(width: 82, height: 82)
                            }
                        }
                        .disabled(!sessionManager.isReadyToCapture)
                        .opacity(sessionManager.isReadyToCapture ? 1.0 : 0.5)
                    }

                    Spacer()

                    VStack(spacing: 8) {
                        // Auto/Manual capture toggle
                        Button {
                            if sessionManager.isAutoCapturing {
                                sessionManager.stopAutoCapture()
                            } else {
                                sessionManager.startAutoCapture()
                            }
                        } label: {
                            Text(sessionManager.isAutoCapturing ? "AUTO" : "MAN")
                                .font(.caption2.weight(.bold))
                                .foregroundColor(sessionManager.isAutoCapturing ? .black : .white)
                                .frame(width: 44, height: 44)
                                .background(
                                    sessionManager.isAutoCapturing
                                        ? AnyShapeStyle(.green)
                                        : AnyShapeStyle(.ultraThinMaterial)
                                )
                                .clipShape(Circle())
                        }

                        // Visualization toggles
                        VisToggle(icon: "square.3.layers.3d", active: sessionManager.showMesh) {
                            sessionManager.toggleMesh()
                        }
                        VisToggle(icon: "circle.dotted", active: sessionManager.showPointCloud) {
                            sessionManager.togglePointCloud()
                        }
                        VisToggle(
                            icon: sessionManager.showDepthOverlay ? "xmark" : "camera.filters",
                            active: sessionManager.showDepthOverlay
                        ) {
                            sessionManager.toggleDepthOverlay()
                        }

                        // Calibration
                        Button { showCalibration = true } label: {
                            Image(systemName: "scope")
                                .font(.title3)
                                .foregroundColor(.white)
                                .frame(width: 44, height: 44)
                                .background(.ultraThinMaterial)
                                .clipShape(Circle())
                        }

                        // End session
                        Button {
                            Task { await sessionManager.endSession() }
                        } label: {
                            Text("End")
                                .font(.subheadline.weight(.semibold))
                                .foregroundColor(.white)
                                .frame(width: 60, height: 60)
                                .background(.red.opacity(0.8))
                                .clipShape(RoundedRectangle(cornerRadius: 8))
                        }
                    }
                }
                .padding(.horizontal, 24)
                .padding(.bottom, 30)
            }
        }
        .sheet(isPresented: $showCalibration) {
            NavigationStack {
                CalibrationView()
            }
        }
        .onChange(of: sessionManager.captureFlash) { _, flashing in
            guard flashing else { return }
            withAnimation(.easeIn(duration: 0.05)) { showFlash = true }
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
                withAnimation(.easeOut(duration: 0.2)) { showFlash = false }
                sessionManager.captureFlash = false
            }
        }
    }

    private var showRetryButton: Bool {
        guard let status = sessionManager.cameraStatus else { return false }
        return status.contains("No cameras") || status.contains("disconnected")
    }

    private var cameraIcon: String {
        guard let status = sessionManager.cameraStatus else { return "camera" }
        if status.contains("Connecting") || status.contains("Retrying") {
            return "antenna.radiowaves.left.and.right"
        }
        if status.contains("No cameras") || status.contains("disconnected") {
            return "exclamationmark.triangle"
        }
        return "camera.fill"
    }

    private var cameraStatusColor: Color {
        guard let status = sessionManager.cameraStatus else { return .white }
        if status.contains("Connecting") || status.contains("Retrying") { return .yellow }
        if status.contains("No cameras") || status.contains("disconnected") { return .red }
        if status.contains("/") { return .yellow }
        return .green
    }
}

private struct ThumbnailView: View {
    let image: UIImage
    let label: String

    var body: some View {
        ZStack(alignment: .bottomLeading) {
            Image(uiImage: image)
                .resizable()
                .aspectRatio(contentMode: .fill)
                .frame(width: 80, height: 80)
                .clipShape(RoundedRectangle(cornerRadius: 8))
                .overlay(
                    RoundedRectangle(cornerRadius: 8)
                        .stroke(.white, lineWidth: 2)
                )
                .shadow(radius: 4)

            Text(label)
                .font(.system(size: 9, weight: .bold))
                .foregroundColor(.white)
                .padding(.horizontal, 3)
                .padding(.vertical, 1)
                .background(.black.opacity(0.6))
                .cornerRadius(3)
                .padding(3)
        }
    }
}

private struct VisToggle: View {
    let icon: String
    let active: Bool
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            Image(systemName: icon)
                .font(.callout)
                .foregroundColor(active ? .yellow : .white)
                .frame(width: 40, height: 40)
                .background(.ultraThinMaterial)
                .overlay(active ? Color.yellow.opacity(0.2) : Color.clear)
                .clipShape(Circle())
        }
    }
}

private struct TrackingStateBadge: View {
    let state: ARCamera.TrackingState

    var body: some View {
        HStack(spacing: 6) {
            Circle()
                .fill(color)
                .frame(width: 10, height: 10)
            Text(label)
                .font(.caption.weight(.medium))
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 6)
        .background(.ultraThinMaterial)
        .cornerRadius(8)
    }

    private var color: Color {
        switch state {
        case .normal:
            return .green
        case .limited:
            return .yellow
        case .notAvailable:
            return .red
        @unknown default:
            return .gray
        }
    }

    private var label: String {
        switch state {
        case .normal:
            return "Tracking"
        case .limited(let reason):
            switch reason {
            case .initializing:
                return "Initializing"
            case .excessiveMotion:
                return "Too fast"
            case .insufficientFeatures:
                return "Low features"
            case .relocalizing:
                return "Relocalizing"
            @unknown default:
                return "Limited"
            }
        case .notAvailable:
            return "Not available"
        @unknown default:
            return "Unknown"
        }
    }
}
