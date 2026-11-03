import Foundation
import simd

/// Records ARKit poses over time for trajectory export.
/// Thread-safe: recordPose is called from the ARKit delegate thread
/// while export/interpolate may be called from @MainActor.
final class TrajectoryRecorder {
    private var poses: [(timestamp: Double, pose: simd_float4x4)] = []
    private let lock = NSLock()

    func recordPose(timestamp: Double, pose: simd_float4x4) {
        lock.lock()
        if let prev = poses.last?.timestamp,
           timestamp - prev < 1e-6
        {
            // skip near-duplicate timestamp
        } else {
            poses.append(
                (timestamp: timestamp, pose: pose)
            )
        }
        lock.unlock()
    }

    /// Interpolates a pose at an arbitrary timestamp using lerp/slerp.
    func interpolatePose(at timestamp: Double) -> simd_float4x4? {
        lock.lock()
        let snapshot = poses
        lock.unlock()

        guard snapshot.count >= 2 else { return snapshot.first?.pose }

        guard let afterIdx = snapshot.firstIndex(where: { $0.timestamp >= timestamp }) else {
            return snapshot.last?.pose
        }
        guard afterIdx > 0 else { return snapshot.first?.pose }

        let before = snapshot[afterIdx - 1]
        let after = snapshot[afterIdx]
        let t = Float((timestamp - before.timestamp) / (after.timestamp - before.timestamp))

        return PoseInterpolation.interpolate(from: before.pose, to: after.pose, t: t)
    }

    func nearestTimestamp(to target: Double) -> Double? {
        lock.lock()
        let snapshot = poses
        lock.unlock()
        guard !snapshot.isEmpty else { return nil }
        var lo = 0, hi = snapshot.count - 1
        while lo < hi {
            let mid = (lo + hi) / 2
            if snapshot[mid].timestamp < target { lo = mid + 1 }
            else { hi = mid }
        }
        if lo == 0 { return snapshot[0].timestamp }
        let before = snapshot[lo - 1].timestamp
        let after = snapshot[lo].timestamp
        return (target - before) <= (after - target)
            ? before : after
    }

    func export() -> TrajectoryData {
        lock.lock()
        let snapshot = poses
        lock.unlock()

        return TrajectoryData(
            poses: snapshot.map { entry in
                PoseEntry(
                    timestamp: entry.timestamp,
                    transform: [entry.pose.columns.0, entry.pose.columns.1,
                                entry.pose.columns.2, entry.pose.columns.3]
                        .map { [$0.x, $0.y, $0.z, $0.w] }
                )
            }
        )
    }
}
