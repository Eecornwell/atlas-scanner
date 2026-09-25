import SwiftUI
import ARKit

struct CaptureOverlayView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager
    @State private var showFlash = false

    var body: some View {
        ZStack {
            // Capture flash
            if showFlash {
                Color.white
                    .ignoresSafeArea()
                    .allowsHitTesting(false)
                    .transition(.opacity)
            }

            VStack {
                // Top bar: tracking state + scan count
                HStack {
                    TrackingStateBadge(
                        state: sessionManager.arkitCapture.trackingState
                    )

                    Spacer()

                    Text("Scans: \(sessionManager.scanCount)")
                        .font(.system(.headline, design: .monospaced))
                        .padding(.horizontal, 12)
                        .padding(.vertical, 6)
                        .background(.ultraThinMaterial)
                        .cornerRadius(8)

                    if let status = sessionManager.cameraStatus {
                        Label(status, systemImage: cameraIcon)
                            .font(.caption.weight(.medium))
                            .foregroundColor(cameraStatusColor)
                            .padding(.horizontal, 10)
                            .padding(.vertical, 6)
                            .background(.ultraThinMaterial)
                            .cornerRadius(8)
                    }
                }
                .padding(.horizontal)
                .padding(.top, 8)

                Spacer()

                // Bottom: capture button + end session
                HStack(alignment: .bottom) {
                    // Thumbnail
                    if let thumb = sessionManager.lastCapturedThumbnail {
                        Image(uiImage: thumb)
                            .resizable()
                            .aspectRatio(contentMode: .fill)
                            .frame(width: 60, height: 60)
                            .clipShape(RoundedRectangle(cornerRadius: 8))
                            .overlay(
                                RoundedRectangle(cornerRadius: 8)
                                    .stroke(.white, lineWidth: 2)
                            )
                            .shadow(radius: 4)
                    } else {
                        Color.clear.frame(width: 60, height: 60)
                    }

                    Spacer()

                    // Capture button
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

                    Spacer()

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
                .padding(.horizontal, 24)
                .padding(.bottom, 30)
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

    private var cameraIcon: String {
        guard let status = sessionManager.cameraStatus else { return "camera" }
        if status.contains("Connecting") { return "antenna.radiowaves.left.and.right" }
        if status.contains("No cameras") { return "camera.badge.ellipsis" }
        return "camera.fill"
    }

    private var cameraStatusColor: Color {
        guard let status = sessionManager.cameraStatus else { return .white }
        if status.contains("Connecting") { return .yellow }
        if status.contains("No cameras") { return .red }
        if status.contains("/") { return .yellow }
        return .green
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
