import SwiftUI

struct SessionStatusView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Circle()
                    .fill(sessionManager.isSessionActive ? .green : .gray)
                    .frame(width: 12, height: 12)
                Text(sessionManager.isSessionActive ? "Session Active" : "No Active Session")
                    .font(.headline)
            }

            if sessionManager.isSessionActive {
                Text("Scans: \(sessionManager.scanCount)")
                    .font(.subheadline)
                Text("Cameras: \(sessionManager.connectedCameraCount)")
                    .font(.subheadline)
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding()
        .background(.ultraThinMaterial)
        .cornerRadius(12)
    }
}
