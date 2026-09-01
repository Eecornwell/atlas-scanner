import UIKit

/// Loads and manages per-camera masks for occlusion filtering.
/// Masks exclude mount hardware, self-view, nadir, and cross-camera occlusion.
/// Source: session's `calibration/masks/<name>.png`, falling back to app bundle.
final class MaskManager {
    private var cameraMasks: [String: UIImage] = [:]
    private var iphoneMask: UIImage?
    private var sessionDirectory: URL?

    func loadMasks(for cameras: [CameraInstance], sessionDirectory: URL) {
        self.sessionDirectory = sessionDirectory
        for camera in cameras {
            if let maskPath = camera.maskPath {
                // maskPath from multi_camera.yaml is relative, e.g. "masks/insta360_primary.png"
                // Resolve against session calibration/ directory
                let name = URL(fileURLWithPath: maskPath).deletingPathExtension().lastPathComponent
                cameraMasks[camera.id] = loadMask(named: name)
            }
        }
        iphoneMask = loadMask(named: "iphone_wide")
    }

    func mask(forCamera cameraId: String) -> UIImage? {
        cameraMasks[cameraId]
    }

    func iphoneCameraMask() -> UIImage? {
        iphoneMask
    }

    // MARK: - Private

    private func loadMask(named name: String) -> UIImage? {
        // 1. Session calibration/masks/ directory (persisted across sessions)
        if let sessionDir = sessionDirectory {
            let url = sessionDir
                .appendingPathComponent("calibration/masks")
                .appendingPathComponent("\(name).png")
            if let data = try? Data(contentsOf: url), let img = UIImage(data: data) {
                return img
            }
        }

        // 2. Shared atlas_sessions/masks/ in Documents (user-placed masks)
        if let documentsDir = FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first {
            let url = documentsDir
                .appendingPathComponent("atlas_sessions/masks")
                .appendingPathComponent("\(name).png")
            if let data = try? Data(contentsOf: url), let img = UIImage(data: data) {
                return img
            }
        }

        // 3. App bundle (default masks shipped with the app)
        if let img = UIImage(named: name) {
            return img
        }

        return nil
    }
}
