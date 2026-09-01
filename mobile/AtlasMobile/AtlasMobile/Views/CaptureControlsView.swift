import SwiftUI
import UIKit

struct CaptureControlsView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager
    @State private var isSharing = false

    var body: some View {
        VStack(spacing: 16) {
            if !sessionManager.isSessionActive {
                Button("Start Session") {
                    Task { await sessionManager.startSession() }
                }
                .buttonStyle(.borderedProminent)
                .controlSize(.large)

                if let dir = sessionManager.sessionDirectory {
                    Button("Share Last Session") { isSharing = true }
                        .buttonStyle(.bordered)
                        .controlSize(.large)
                        .sheet(isPresented: $isSharing) {
                            ShareSheet(url: dir)
                        }
                }
            } else {
                Button("Capture Scan") {
                    Task { await sessionManager.captureScan() }
                }
                .buttonStyle(.borderedProminent)
                .controlSize(.large)
                .disabled(!sessionManager.isReadyToCapture)

                Button("End Session") {
                    Task { await sessionManager.endSession() }
                }
                .buttonStyle(.bordered)
                .controlSize(.large)
            }
        }
        .sheet(isPresented: $sessionManager.showExportSheet) {
            if let dir = sessionManager.sessionDirectory {
                SessionExportView(sessionDirectory: dir)
            }
        }
    }
}

private struct ShareSheet: UIViewControllerRepresentable {
    let url: URL
    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: [url], applicationActivities: nil)
    }
    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}
