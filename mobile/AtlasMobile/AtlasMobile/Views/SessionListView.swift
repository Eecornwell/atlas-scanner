import SwiftUI

struct SessionListView: View {
    @EnvironmentObject var sessionManager: CaptureSessionManager
    @State private var sessions: [SessionSummary] = []

    var body: some View {
        List {
            if sessions.isEmpty {
                Text("No sessions yet")
                    .foregroundColor(.secondary)
            } else {
                ForEach(sessions) { session in
                    SessionRow(session: session)
                }
            }
        }
        .navigationTitle("Sessions")
        .onAppear { sessions = loadSessions() }
    }

    private func loadSessions() -> [SessionSummary] {
        SessionDirectory.listSessions().map { url in
            let name = url.lastPathComponent
            let scanCount = (try? FileManager.default
                .contentsOfDirectory(atPath: url.appendingPathComponent("raw/iphone").path)
                .filter { $0.hasPrefix("scan_") }.count) ?? 0
            let created = (try? url.resourceValues(forKeys: [.creationDateKey]).creationDate) ?? Date.distantPast
            let size = directorySize(url)
            return SessionSummary(url: url, name: name, scanCount: scanCount,
                                  created: created, sizeBytes: size)
        }
    }

    private func directorySize(_ url: URL) -> Int64 {
        guard let enumerator = FileManager.default.enumerator(
            at: url, includingPropertiesForKeys: [.fileSizeKey], options: [.skipsHiddenFiles]
        ) else { return 0 }
        return enumerator.compactMap { ($0 as? URL).flatMap {
            try? $0.resourceValues(forKeys: [.fileSizeKey]).fileSize
        }}.reduce(0) { $0 + Int64($1) }
    }
}

// MARK: - Row

private struct SessionRow: View {
    let session: SessionSummary
    @State private var showExport = false

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(session.name)
                .font(.headline)
                .lineLimit(1)
            HStack(spacing: 12) {
                Label("\(session.scanCount) scans", systemImage: "camera")
                Label(session.formattedDate, systemImage: "calendar")
                Label(session.formattedSize, systemImage: "internaldrive")
            }
            .font(.caption)
            .foregroundColor(.secondary)
        }
        .padding(.vertical, 4)
        .contextMenu {
            Button {
                showExport = true
            } label: {
                Label("Export / Process", systemImage: "square.and.arrow.up")
            }
        }
        .sheet(isPresented: $showExport) {
            SessionExportView(sessionDirectory: session.url)
        }
    }
}

// MARK: - Model

private struct SessionSummary: Identifiable {
    let url: URL
    let name: String
    let scanCount: Int
    let created: Date
    let sizeBytes: Int64

    var id: String { url.path }

    var formattedDate: String {
        let f = DateFormatter()
        f.dateStyle = .short
        f.timeStyle = .short
        return f.string(from: created)
    }

    var formattedSize: String {
        ByteCountFormatter.string(fromByteCount: sizeBytes, countStyle: .file)
    }
}
