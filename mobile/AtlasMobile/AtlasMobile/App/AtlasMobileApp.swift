import SwiftUI
import INSCameraSDK
import INSCameraServiceSDK

final class SDKLogForwarder: NSObject, INSCameraSDKLoggerProtocol {
    static let shared = SDKLogForwarder()
    var messages: [String] = []

    func logError(_ message: String, filePath: String, funcName: String, lineNum: Int) {
        messages.append("[SDK ERROR] \(funcName): \(message)")
    }

    func logWarning(_ message: String, filePath: String, funcName: String, lineNum: Int) {
        messages.append("[SDK WARN] \(funcName): \(message)")
    }

    func logInfo(_ message: String, filePath: String, funcName: String, lineNum: Int) {
        messages.append("[SDK INFO] \(funcName): \(message)")
    }

    func logDebug(_ message: String, filePath: String, funcName: String, lineNum: Int) {
        messages.append("[SDK DEBUG] \(funcName): \(message)")
    }

    func logCrash(_ message: String, filePath: String, funcName: String, lineNum: Int) {
        messages.append("[SDK CRASH] \(funcName): \(message)")
    }
}

@main
struct AtlasMobileApp: App {
    @StateObject private var sessionManager = CaptureSessionManager()

    init() {
        INSCameraSDKLogger.shared().logLevel = INSCameraSDKLogLevel.debug
        INSCameraSDKLogger.shared().logDelegate = SDKLogForwarder.shared
    }

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(sessionManager)
                .onAppear {
                    let socket = INSCameraManager.socket()
                    socket.autoReconnect = true
                    if socket.cameraState != .connected {
                        socket.setup()
                    }
                }
        }
    }
}
