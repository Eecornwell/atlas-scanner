import Foundation
import INSCameraSDK
import INSCoreMedia

/// Keeps the Wi-Fi camera connection alive the way the SDK sample's
/// RootViewController does (INSCameraSDKSample-bluetooth/common/
/// RootViewController.swift): one app-wide observer on
/// INSCameraManager.socket().cameraState. On every .connected,
/// including the SDK's own auto-reconnects, it starts the 0.5 s
/// heartbeat and, for the X6 (SDK name C9), sends
/// setAppAccessFileState(.liveView). The sample's comment says that
/// call wakes the X6 sensor out of low power. On .connectFailed or
/// .noConnection it stops the heartbeat.
///
/// Everything here runs on the main queue.
final class Insta360ConnectionMonitor {
    static let shared = Insta360ConnectionMonitor()

    /// Receives status lines. Insta360CaptureManager points this at its
    /// own log. Set and called on the main queue.
    var onLogMessage: ((String) -> Void)?

    private var stateToken: NSKeyValueObservation?
    private var heartbeat: DispatchSourceTimer?
    /// Counts connections, so a camera-name retry left over from an
    /// earlier connection doesn't send liveView.
    private var connection = 0

    private init() {}

    /// Starts observing. Call once at launch, after the INSCameraManager
    /// setup calls.
    func start() {
        guard stateToken == nil else { return }
        stateToken = INSCameraManager.socket().observe(
            \.cameraState, options: [.initial, .new]
        ) { [weak self] manager, _ in
            let state = manager.cameraState
            DispatchQueue.main.async {
                self?.handle(state)
            }
        }
    }

    private func handle(_ state: INSCameraState) {
        switch state {
        case .connected:
            connection += 1
            startHeartbeat()
            wakeX6IfNeeded(connection: connection, attempt: 0)
        case .connectFailed, .noConnection:
            stopHeartbeat()
        default:
            // .found and .synchronized: the sample leaves the heartbeat
            // as it is.
            break
        }
    }

    /// Same timer as the sample's GCDTimer: main queue, 0.5 s, 50 ms
    /// leeway. A GCD timer keeps firing while a scroll view is being
    /// dragged; a Timer in the default run-loop mode doesn't.
    private func startHeartbeat() {
        guard heartbeat == nil else { return }
        let timer = DispatchSource.makeTimerSource(queue: .main)
        timer.schedule(
            deadline: .now(),
            repeating: 0.5,
            leeway: .milliseconds(50)
        )
        timer.setEventHandler {
            INSCameraManager.shared().commandManager
                .sendHeartbeats(with: nil)
        }
        timer.resume()
        heartbeat = timer
        log("heartbeat started")
    }

    private func stopHeartbeat() {
        guard let timer = heartbeat else { return }
        timer.cancel()
        heartbeat = nil
        log("heartbeat stopped")
    }

    /// The camera name can arrive just after the socket connects, so
    /// this checks every 0.25 s for up to 3 s.
    private func wakeX6IfNeeded(connection: Int, attempt: Int) {
        guard connection == self.connection,
              INSCameraManager.socket().cameraState == .connected
        else { return }
        guard let name = INSCameraManager.shared().currentCamera?.name,
              !name.isEmpty
        else {
            if attempt < 12 {
                DispatchQueue.main.asyncAfter(
                    deadline: .now() + 0.25
                ) { [weak self] in
                    self?.wakeX6IfNeeded(
                        connection: connection,
                        attempt: attempt + 1
                    )
                }
            } else {
                log("camera name unknown, X6 live view not sent")
            }
            return
        }
        guard name == kInsta360CameraNameC9 else { return }
        INSCameraManager.shared().commandManager
            .setAppAccessFileState(.liveView) { [weak self] error in
                DispatchQueue.main.async {
                    self?.log(
                        "X6 live view on connect: "
                            + (error.map { "\($0)" } ?? "ok")
                    )
                }
            }
    }

    private func log(_ message: String) {
        print("[Insta360] \(message)")
        onLogMessage?(message)
    }
}
