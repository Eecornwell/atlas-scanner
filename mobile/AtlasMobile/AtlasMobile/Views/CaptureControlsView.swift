import SwiftUI
import UIKit

struct CaptureControlsView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager
    @State private var isSharing = false
    @State private var isZipping = false
    @State private var shareZipURL: URL?

    var body: some View {
        VStack(spacing: 16) {
            Button("Start Session") {
                Task { await sessionManager.startSession() }
            }
            .buttonStyle(.borderedProminent)
            .controlSize(.large)

            if let dir = sessionManager.sessionDirectory {
                Button(isZipping ? "Zipping…" : "Share Last Session") {
                    Task { await zipAndShare(dir) }
                }
                .buttonStyle(.bordered)
                .controlSize(.large)
                .disabled(isZipping)
                .sheet(isPresented: $isSharing) {
                    if let zipURL = shareZipURL {
                        ShareSheet(url: zipURL)
                    }
                }
            }

            if let error = sessionManager.exportError {
                Label(error, systemImage: "exclamationmark.triangle")
                    .foregroundColor(.red)
                    .font(.caption)
            }
        }
    }

    private func zipAndShare(_ dir: URL) async {
        isZipping = true
        let url = await Task.detached(priority: .userInitiated) {
            let zipURL = FileManager.default.temporaryDirectory
                .appendingPathComponent("\(dir.lastPathComponent).zip")
            try? FileManager.default.removeItem(at: zipURL)
            return ZipWriter.write(sourceDir: dir, to: zipURL)
                ? zipURL : nil
        }.value
        isZipping = false
        guard let url else { return }
        shareZipURL = url
        isSharing = true
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
