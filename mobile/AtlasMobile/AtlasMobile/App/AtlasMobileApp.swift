import SwiftUI
import INSCameraSDK

@main
struct AtlasMobileApp: App {
    @StateObject private var sessionManager = CaptureSessionManager()

    init() {
        INSCameraManager.shared().setup()
    }

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(sessionManager)
        }
    }
}
