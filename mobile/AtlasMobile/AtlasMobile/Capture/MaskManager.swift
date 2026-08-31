import UIKit

/// Loads and manages per-camera masks for occlusion filtering.
/// Masks exclude mount hardware, self-view, nadir, and cross-camera occlusion.
final class MaskManager {
    private var cameraMasks: [String: UIImage] = [:]
    private var iphoneMask: UIImage?

    func loadMasks(for cameras: [CameraInstance]) {
        for camera in cameras {
            if let maskPath = camera.maskPath {
                cameraMasks[camera.id] = loadMask(named: maskPath)
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

    private func loadMask(named name: String) -> UIImage? {
        // TODO: Load from session calibration/masks directory
        // Masks are single-channel PNG (0 = excluded, 255 = valid)
        guard let documentsDir = FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first else { return nil }

        let maskURL = documentsDir
            .appendingPathComponent("masks")
            .appendingPathComponent("\(name).png")

        guard let data = try? Data(contentsOf: maskURL) else { return nil }
        return UIImage(data: data)
    }
}
