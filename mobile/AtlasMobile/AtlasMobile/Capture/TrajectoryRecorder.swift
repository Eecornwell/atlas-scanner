import Foundation
import simd

/// Records ARKit poses over time for trajectory export.
/// Mirrors atlas-scanner's enhanced_trajectory_recorder.py.
final class TrajectoryRecorder {
    private var poses: [(timestamp: Double, pose: simd_float4x4)] = []

    func recordPose(timestamp: Double, pose: simd_float4x4) {
        poses.append((timestamp: timestamp, pose: pose))
    }

    /// Interpolates a pose at an arbitrary timestamp using lerp/slerp.
    func interpolatePose(at timestamp: Double) -> simd_float4x4? {
        guard poses.count >= 2 else { return poses.first?.pose }

        guard let afterIdx = poses.firstIndex(where: { $0.timestamp >= timestamp }) else {
            return poses.last?.pose
        }
        guard afterIdx > 0 else { return poses.first?.pose }

        let before = poses[afterIdx - 1]
        let after = poses[afterIdx]
        let t = Float((timestamp - before.timestamp) / (after.timestamp - before.timestamp))

        return PoseInterpolation.interpolate(from: before.pose, to: after.pose, t: t)
    }

    func export() -> TrajectoryData {
        TrajectoryData(
            poses: poses.map { entry in
                PoseEntry(
                    timestamp: entry.timestamp,
                    transform: entry.pose.columns.map { [$0.x, $0.y, $0.z, $0.w] }
                )
            }
        )
    }
}
