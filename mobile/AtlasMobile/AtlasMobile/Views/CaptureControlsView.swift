import SwiftUI
import UIKit

struct CaptureControlsView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager
    @State private var showExport = false

    var body: some View {
        VStack(spacing: 16) {
            Button("Start Session") {
                Task { await sessionManager.startSession() }
            }
            .buttonStyle(.borderedProminent)
            .controlSize(.large)

            if let dir = sessionManager.sessionDirectory {
                Button("Process Last Session") {
                    showExport = true
                }
                .buttonStyle(.bordered)
                .controlSize(.large)
                .sheet(isPresented: $showExport) {
                    SessionExportView(sessionDirectory: dir)
                }
            }

            if let error = sessionManager.exportError {
                Label(error, systemImage: "exclamationmark.triangle")
                    .foregroundColor(.red)
                    .font(.caption)
            }
        }
    }
}

private struct ShareSheet: UIViewControllerRepresentable {
    let url: URL
    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(
            activityItems: [url],
            applicationActivities: nil
        )
    }
    func updateUIViewController(
        _ uiViewController: UIActivityViewController,
        context: Context
    ) {}
}
