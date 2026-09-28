import SwiftUI

struct ContentView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager
    @State private var showCalibration = false
    @State private var configuredCameraCount = 0

    var body: some View {
        NavigationStack {
            ZStack {
                if sessionManager.isSessionActive,
                   let session = sessionManager.arkitCapture.session {
                    ARCameraView(
                        session: session,
                        showMesh: sessionManager.showMesh,
                        showPointCloud: sessionManager.showPointCloud
                    )
                    .ignoresSafeArea()

                    CaptureOverlayView()
                } else {
                    homeView
                }
            }
            .navigationTitle(sessionManager.isSessionActive ? "" : "ATLAS Mobile Capture")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar(sessionManager.isSessionActive ? .hidden : .visible, for: .navigationBar)
            .sheet(isPresented: $sessionManager.showExportSheet) {
                if let dir = sessionManager.sessionDirectory {
                    SessionExportView(sessionDirectory: dir)
                }
            }
            .sheet(isPresented: $showCalibration) {
                NavigationStack {
                    CalibrationView()
                }
            }
        }
    }

    private var homeView: some View {
        VStack(spacing: 20) {
            SessionStatusView()

            if configuredCameraCount > 0 {
                Label(
                    "\(configuredCameraCount) Insta360 camera\(configuredCameraCount == 1 ? "" : "s") configured",
                    systemImage: "camera.fill"
                )
                .font(.subheadline)
                .foregroundColor(.green)
            } else {
                Label("No external cameras — iPhone only", systemImage: "iphone")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
            }

            Spacer()

            CaptureControlsView()

            Spacer()

            NavigationLink("Sessions") {
                SessionListView()
            }

            Button("Calibration") {
                Task {
                    await sessionManager.startSession()
                    showCalibration = true
                }
            }

            NavigationLink("Camera Settings") {
                CameraSettingsView()
            }
        }
        .padding()
        .onAppear {
            configuredCameraCount = MultiCameraConfig.loadFromDeviceOrBundle().cameras.count
        }
    }
}
