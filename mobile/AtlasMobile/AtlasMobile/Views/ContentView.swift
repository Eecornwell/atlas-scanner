import SwiftUI

struct ContentView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager

    var body: some View {
        NavigationStack {
            ZStack {
                if sessionManager.isSessionActive,
                   let session = sessionManager.arkitCapture.session {
                    ARCameraView(session: session)
                        .ignoresSafeArea()

                    CaptureOverlayView()
                } else {
                    homeView
                }
            }
            .navigationTitle(sessionManager.isSessionActive ? "" : "ATLAS Mobile")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar(sessionManager.isSessionActive ? .hidden : .visible, for: .navigationBar)
            .sheet(isPresented: $sessionManager.showExportSheet) {
                if let dir = sessionManager.sessionDirectory {
                    SessionExportView(sessionDirectory: dir)
                }
            }
        }
    }

    private var homeView: some View {
        VStack(spacing: 20) {
            SessionStatusView()

            Spacer()

            CaptureControlsView()

            Spacer()

            NavigationLink("Sessions") {
                SessionListView()
            }

            NavigationLink("Calibration") {
                CalibrationView()
            }
        }
        .padding()
    }
}
