import Foundation
import INSCameraSDK

/// Camera WiFi Direct defaults — Insta360 AP mode address.
private let kCameraHost = "192.168.42.1"
private let kCameraPort: UInt16 = 6666
private let kHeartbeatInterval: TimeInterval = 0.5
private let kClockSyncSamples = 10

/// Represents a single connected Insta360 camera with its calibration and state.
final class CameraInstance: NSObject, Identifiable {
    let id: String
    let model: String
    let serial: String
    let extrinsic: RigidTransform
    let maskPath: String?

    private(set) var clockOffset: ClockOffset?
    private(set) var isConnected = false

    /// Pending media URIs queued for download (populated by capture()).
    private var pendingURIs: [(scanIndex: Int, uri: String)] = []
    private var heartbeatTimer: Timer?
    private var kvoToken: NSKeyValueObservation?

    init(config: CameraConfig) {
        self.id = config.id
        self.model = config.model
        self.serial = config.serial
        self.extrinsic = config.extrinsic
        self.maskPath = config.mask
    }

    // MARK: - Connection

    func connect() async -> Bool {
        let device = INSSocketDevice(host: kCameraHost, port: kCameraPort)
        INSCameraManager.socket().currentCamera = device as? (any INSCameraDevice)

        return await withCheckedContinuation { continuation in
            kvoToken = INSCameraManager.socket().observe(
                \.cameraState,
                options: [.new]
            ) { [weak self] manager, change in
                guard let self, let rawValue = change.newValue else { return }
                let state = INSCameraState(rawValue: rawValue.uintValue) ?? .noConnection
                switch state {
                case .connected:
                    self.kvoToken = nil
                    self.isConnected = true
                    self.startHeartbeat()
                    continuation.resume(returning: true)
                case .connectFailed:
                    self.kvoToken = nil
                    continuation.resume(returning: false)
                default:
                    break
                }
            }
            INSCameraManager.socket().setup()
        }
    }

    func disconnect() async {
        stopHeartbeat()
        kvoToken = nil
        INSCameraManager.socket().shutdown()
        isConnected = false
    }

    // MARK: - Clock offset

    /// Estimates clock offset using the SDK's built-in sync (median of N samples).
    func calibrateClockOffset() async -> ClockOffset {
        return await withCheckedContinuation { continuation in
            INSCameraManager.socket().commandsImpl.syncTimeMsToCamera(
                withTryCount: kClockSyncSamples,
                dTimeMsMax: 200
            ) { [weak self] dTimeMs, error in
                let offset = ClockOffset(
                    offsetMs: error == nil ? Double(dTimeMs) : 0.0,
                    sampleCount: error == nil ? kClockSyncSamples : 0,
                    stdDevMs: 0.0
                )
                self?.clockOffset = offset
                continuation.resume(returning: offset)
            }
        }
    }

    // MARK: - Capture

    func capture(arkitTimestamp: Double, scanIndex: Int) async -> Insta360CaptureResult? {
        guard isConnected else { return nil }

        return await withCheckedContinuation { continuation in
            let options = INSTakePictureOptions()
            INSCameraManager.socket().commandsImpl.takePicture(with: options) { [weak self] error, photoInfo in
                guard let self, error == nil, let uri = photoInfo?.uri else {
                    continuation.resume(returning: nil)
                    return
                }
                let insta360Ts = Date().timeIntervalSince1970
                self.pendingURIs.append((scanIndex: scanIndex, uri: uri))
                continuation.resume(returning: Insta360CaptureResult(
                    cameraId: self.id,
                    scanIndex: scanIndex,
                    arkitTimestamp: arkitTimestamp,
                    insta360Timestamp: insta360Ts,
                    mediaIdentifier: uri
                ))
            }
        }
    }

    // MARK: - Download

    func downloadPending(into directory: URL) async -> [Insta360DownloadResult] {
        guard isConnected, !pendingURIs.isEmpty else { return [] }
        let toDownload = pendingURIs
        pendingURIs.removeAll()

        return await withTaskGroup(of: Insta360DownloadResult?.self) { group in
            for item in toDownload {
                group.addTask {
                    await self.downloadOne(scanIndex: item.scanIndex, uri: item.uri, into: directory)
                }
            }
            var results: [Insta360DownloadResult] = []
            for await result in group {
                if let r = result { results.append(r) }
            }
            return results
        }
    }

    // MARK: - Private

    private func downloadOne(scanIndex: Int, uri: String, into directory: URL) async -> Insta360DownloadResult? {
        let destURL = directory
            .appendingPathComponent(id)
            .appendingPathComponent(String(format: "scan_%03d.jpg", scanIndex))

        try? FileManager.default.createDirectory(
            at: destURL.deletingLastPathComponent(),
            withIntermediateDirectories: true
        )

        return await withCheckedContinuation { continuation in
            INSCameraManager.socket().commandsImpl.fetchResource(
                withURI: uri,
                toLocalFile: destURL,
                progress: nil
            ) { error in
                if error != nil {
                    continuation.resume(returning: nil)
                    return
                }
                continuation.resume(returning: Insta360DownloadResult(
                    cameraId: self.id,
                    scanIndex: scanIndex,
                    localURL: destURL,
                    mediaType: .equirectangular
                ))
            }
        }
    }

    private func startHeartbeat() {
        heartbeatTimer = Timer.scheduledTimer(withTimeInterval: kHeartbeatInterval, repeats: true) { _ in
            INSCameraManager.socket().commandsImpl.sendHeartbeats(with: nil)
        }
    }

    private func stopHeartbeat() {
        heartbeatTimer?.invalidate()
        heartbeatTimer = nil
    }
}
