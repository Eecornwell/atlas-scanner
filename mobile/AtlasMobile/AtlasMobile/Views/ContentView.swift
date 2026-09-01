import SwiftUI

struct ContentView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager

    var body: some View {
        NavigationStack {
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
            .navigationTitle("Atlas Mobile")
        }
    }
}
