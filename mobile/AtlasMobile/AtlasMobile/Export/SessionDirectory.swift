import Foundation

enum SessionDirectory {
    static func create() -> URL {
        let documentsDir = FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first!

        let sessionsDir = documentsDir.appendingPathComponent("atlas_sessions")
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyy-MM-dd_HH-mm-ss"
        let sessionName = "session_\(formatter.string(from: Date()))"
        let sessionDir = sessionsDir.appendingPathComponent(sessionName)

        try? FileManager.default.createDirectory(
            at: sessionDir,
            withIntermediateDirectories: true
        )

        return sessionDir
    }

    static func listSessions() -> [URL] {
        let documentsDir = FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first!
        let sessionsDir = documentsDir.appendingPathComponent("atlas_sessions")

        guard let contents = try? FileManager.default.contentsOfDirectory(
            at: sessionsDir,
            includingPropertiesForKeys: [.isDirectoryKey, .creationDateKey],
            options: [.skipsHiddenFiles]
        ) else { return [] }

        let sessions = contents.filter { url in
            let isDir = (try? url.resourceValues(
                forKeys: [.isDirectoryKey]
            ).isDirectory) ?? false
            return isDir
                && url.lastPathComponent.hasPrefix("session_")
        }

        return sessions.sorted { a, b in
            let aDate = (try? a.resourceValues(forKeys: [.creationDateKey]).creationDate) ?? .distantPast
            let bDate = (try? b.resourceValues(forKeys: [.creationDateKey]).creationDate) ?? .distantPast
            return aDate > bDate
        }
    }
}
