import Darwin
import SwiftUI
import UIKit
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

/// Launch setup from the SDK sample's AppDelegate
/// (INSCameraSDKSample-bluetooth/common/AppDelegate.swift), plus the
/// app-wide connection monitor that does what the sample's
/// RootViewController does on every connection.
final class AppDelegate: NSObject, UIApplicationDelegate {
    func application(
        _ application: UIApplication,
        didFinishLaunchingWithOptions launchOptions:
            [UIApplication.LaunchOptionsKey: Any]?
    ) -> Bool {
        INSCameraManager.shared().setup()
        INSCameraManager.socket().autoReconnect = true
        INSCameraManager.shared().autoReconnect = true
        Insta360ConnectionMonitor.shared.start()
        return true
    }
}

@main
struct AtlasMobileApp: App {
    @UIApplicationDelegateAdaptor(AppDelegate.self) private var appDelegate
    @StateObject private var sessionManager = CaptureSessionManager()

    init() {
        // A write to a socket the Insta360 camera has already closed
        // raises SIGPIPE. Its default action ends the app without a
        // crash report. Ignoring it makes the write return an EPIPE
        // error to the caller instead.
        signal(SIGPIPE, SIG_IGN)
        UserDefaults.standard.register(defaults: [
            "multiCameraExport": true,
        ])
        INSCameraSDKLogger.shared().logLevel = INSCameraSDKLogLevel.debug
        INSCameraSDKLogger.shared().logDelegate = SDKLogForwarder.shared
    }

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(sessionManager)
                .onAppear {
                    // The sample calls socket().setup() when the user
                    // taps "Connect Wi-Fi Mode"; this app connects at
                    // launch. autoReconnect is set in AppDelegate.
                    let socket = INSCameraManager.socket()
                    if socket.cameraState != .connected {
                        socket.setup()
                    }
                }
        }
    }
}
