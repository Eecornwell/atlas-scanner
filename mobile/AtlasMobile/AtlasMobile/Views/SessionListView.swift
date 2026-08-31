import SwiftUI

struct SessionListView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager

    var body: some View {
        List {
            // TODO: populate from saved sessions on disk
            Text("No sessions yet")
                .foregroundColor(.secondary)
        }
        .navigationTitle("Sessions")
    }
}
