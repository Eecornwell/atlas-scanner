import SwiftUI
import INSCameraSDK
import INSCameraServiceSDK

final class SDKLogForwarder: NSObject, INSCameraSDKLoggerProtocol {
    static let shared = SDKLogForwarder()
    private var _messages: [String] = []
    private let lock = NSLock()

    var messages: [String] {
        lock.lock()
        defer { lock.unlock() }
        return _messages
    }

    private func append(_ msg: String) {
        lock.lock()
        _messages.append(msg)
        if _messages.count > 200 {
            _messages.removeFirst(
                _messages.count - 200
            )
        }
        lock.unlock()
    }

    func logError(_ message: String, filePath: String, funcName: String, lineNum: Int) {
        append("[SDK ERROR] \(funcName): \(message)")
    }

    func logWarning(_ message: String, filePath: String, funcName: String, lineNum: Int) {
        append("[SDK WARN] \(funcName): \(message)")
    }

    func logInfo(_ message: String, filePath: String, funcName: String, lineNum: Int) {
        append("[SDK INFO] \(funcName): \(message)")
    }

    func logDebug(_ message: String, filePath: String, funcName: String, lineNum: Int) {
        append("[SDK DEBUG] \(funcName): \(message)")
    }

    func logCrash(_ message: String, filePath: String, funcName: String, lineNum: Int) {
        append("[SDK CRASH] \(funcName): \(message)")
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
