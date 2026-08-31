import SwiftUI

struct CaptureControlsView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager

    var body: some View {
        VStack(spacing: 16) {
            if !sessionManager.isSessionActive {
                Button("Start Session") {
                    Task { await sessionManager.startSession() }
                }
                .buttonStyle(.borderedProminent)
                .controlSize(.large)
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
    }
}
