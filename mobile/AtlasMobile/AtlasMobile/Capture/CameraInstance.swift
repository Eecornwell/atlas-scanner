import Foundation

/// Represents a single connected Insta360 camera with its calibration and state.
final class CameraInstance: Identifiable {
    let id: String
    let model: String
    let serial: String
    let extrinsic: RigidTransform
    let maskPath: String?

    private(set) var clockOffset: ClockOffset?
    private(set) var isConnected = false
    private var pendingDownloads: [String] = []

    init(config: CameraConfig) {
        self.id = config.id
        self.model = config.model
        self.serial = config.serial
        self.extrinsic = config.extrinsic
        self.maskPath = config.mask
    }

    func connect() async -> Bool {
        // TODO: Connect to camera via Insta360 SDK
        // 1. WiFi connection to camera SSID
        // 2. Initialize SDK session
        // 3. Calibrate clock offset
        isConnected = true
        return true
    }

    func disconnect() async {
        // TODO: Disconnect SDK session
        isConnected = false
    }

    func calibrateClockOffset(arkitTimestamp: Double) async -> ClockOffset {
        // TODO: Send capture command, measure round-trip, estimate offset
        // Same approach as atlas-scanner: median of N samples
        let offset = ClockOffset(
            offsetMs: 0.0,
            sampleCount: 0,
            stdDevMs: 0.0
        )
        self.clockOffset = offset
        return offset
    }

    func capture(arkitTimestamp: Double) async -> Insta360CaptureResult? {
        guard isConnected else { return nil }

        // TODO: Trigger capture via Insta360 SDK
        // 1. Send capture command
        // 2. Wait for completion callback
        // 3. Record Insta360 timestamp from EXIF/sidecar
        // 4. Compute clock offset sample

        return Insta360CaptureResult(
            cameraId: id,
            arkitTimestamp: arkitTimestamp,
            insta360Timestamp: 0,
            mediaIdentifier: ""
        )
    }

    func downloadPending() async -> [Insta360DownloadResult] {
        // TODO: Download media files from camera via SDK
        return []
    }
}
