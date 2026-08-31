import SwiftUI

@main
struct AtlasMobileApp: App {
    @StateObject private var sessionManager = CaptureSessionManager()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(sessionManager)
        }
    }
}
