import SwiftUI

/// Presented after endSession() — lets the user choose how to process the session.
struct SessionExportView: View {
    let sessionDirectory: URL
    @StateObject private var postProcessor = PostProcessingManager()
    @StateObject private var uploader = HostUploader()
    @AppStorage("hostURL") private var hostURL = "http://192.168.1.100:8765"

    @State private var selectedMode: ExportMode = .colmapOnly
    @State private var isRunning = false
    @State private var isComplete = false
    @State private var isSharing = false
    @State private var isZipping = false
    @State private var shareZipURL: URL?
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            Form {
                // Mode picker
                Section("Export Mode") {
                    ForEach(ExportMode.allCases) { mode in
                        ModeRow(mode: mode, isSelected: selectedMode == mode) {
                            selectedMode = mode
                        }
                    }
                }

                // Host URL (only shown for host mode)
                if selectedMode == .hostProcessing {
                    Section("Host") {
                        HStack {
                            Text("URL")
                                .foregroundColor(.secondary)
                            TextField("http://192.168.1.100:8765", text: $hostURL)
                                .keyboardType(.URL)
                                .autocorrectionDisabled()
                                .textInputAutocapitalization(.never)
                        }
                        Text("Run enhance_server.py on your workstation first.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }

                // On-device model availability
                if selectedMode == .fullOnDevice {
                    Section("Models") {
                        ModelStatusRow("PromptDA",
                                       available: postProcessor.promptDAAvailable)
                        ModelStatusRow("StableNormal",
                                       available: postProcessor.stableNormalAvailable)
                        if !postProcessor.promptDAAvailable || !postProcessor.stableNormalAvailable {
                            Text("Bundle PromptDA.mlpackage and StableNormal.mlpackage " +
                                 "with the app. Run convert_models.py to generate them.")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                }

                // Progress
                if isRunning || isComplete {
                    Section("Progress") {
                        progressSection
                    }
                }

                // Host log
                if selectedMode == .hostProcessing && !uploader.logLines.isEmpty {
                    Section("Host Log") {
                        ScrollView {
                            VStack(alignment: .leading, spacing: 2) {
                                ForEach(uploader.logLines, id: \.self) { line in
                                    Text(line)
                                        .font(.system(.caption2, design: .monospaced))
                                        .foregroundColor(.secondary)
                                }
                            }
                        }
                        .frame(maxHeight: 150)
                    }
                }
            }
            .navigationTitle("Export Session")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Skip") { dismiss() }
                        .disabled(isRunning)
                }
                ToolbarItem(placement: .confirmationAction) {
                    Button(isRunning ? "Running…" : "Export") {
                        Task { await runExport() }
                    }
                    .disabled(isRunning || isComplete)
                }
            }
            .task { await postProcessor.loadModels() }
        }
    }

    // MARK: - Progress section

    @ViewBuilder
    private var progressSection: some View {
        let p = currentProgress
        VStack(alignment: .leading, spacing: 8) {
            Text(p.stage)
                .font(.subheadline)
            ProgressView(value: p.fraction)
            if let err = p.error {
                Label(err, systemImage: "exclamationmark.triangle")
                    .foregroundColor(.red)
                    .font(.caption)
            }
            if p.isComplete {
                Label("Done", systemImage: "checkmark.circle.fill")
                    .foregroundColor(.green)
                Button(isZipping ? "Zipping…" : "Share Session") {
                    Task { await zipAndShare() }
                }
                .buttonStyle(.bordered)
                .disabled(isZipping)
                .sheet(isPresented: $isSharing) {
                    if let zipURL = shareZipURL {
                        ShareSheet(url: zipURL)
                    }
                }
            }
        }
    }

    private var currentProgress: ExportProgress {
        switch selectedMode {
        case .colmapOnly:    return postProcessor.progress
        case .fullOnDevice:  return postProcessor.progress
        case .hostProcessing: return uploader.progress
        }
    }

    // MARK: - Run

    private func zipAndShare() async {
        isZipping = true
        let dir = sessionDirectory
        let url = await Task.detached(priority: .userInitiated) {
            let zipURL = FileManager.default.temporaryDirectory
                .appendingPathComponent("\(dir.lastPathComponent).zip")
            try? FileManager.default.removeItem(at: zipURL)
            return ZipWriter.write(sourceDir: dir, to: zipURL) ? zipURL : nil
        }.value
        isZipping = false
        guard let url else { return }
        shareZipURL = url
        isSharing = true
    }

    private func runExport() async {
        isRunning = true

        switch selectedMode {
        case .colmapOnly:
            let sparseDir = sessionDirectory
                .appendingPathComponent("colmap/sparse/0")
            let camerasExist = FileManager.default
                .fileExists(atPath: sparseDir.appendingPathComponent("cameras.bin").path)

            if camerasExist {
                postProcessor.progress = ExportProgress(
                    stage: "COLMAP model ready",
                    current: 1, total: 1, isComplete: true
                )
            } else {
                postProcessor.progress = ExportProgress(
                    stage: "COLMAP export failed — no output files found",
                    error: "Check that the session has at least one scan with valid pose data."
                )
            }

        case .fullOnDevice:
            await postProcessor.runFullPipeline(sessionDirectory: sessionDirectory)

        case .hostProcessing:
            await uploader.upload(sessionDirectory: sessionDirectory, hostURL: hostURL)
        }

        isRunning = false
        isComplete = true
    }
}

// MARK: - Sub-views

private struct ModeRow: View {
    let mode: ExportMode
    let isSelected: Bool
    let onTap: () -> Void

    var body: some View {
        Button(action: onTap) {
            HStack(alignment: .top, spacing: 12) {
                Image(systemName: mode.systemImage)
                    .frame(width: 28)
                    .foregroundColor(isSelected ? .accentColor : .secondary)
                VStack(alignment: .leading, spacing: 3) {
                    Text(mode.rawValue)
                        .font(.headline)
                        .foregroundColor(.primary)
                    Text(mode.description)
                        .font(.caption)
                        .foregroundColor(.secondary)
                        .fixedSize(horizontal: false, vertical: true)
                }
                Spacer()
                if isSelected {
                    Image(systemName: "checkmark")
                        .foregroundColor(.accentColor)
                }
            }
            .padding(.vertical, 4)
        }
        .buttonStyle(.plain)
    }
}

private struct ModelStatusRow: View {
    let name: String
    let available: Bool
    init(_ name: String, available: Bool) {
        self.name = name
        self.available = available
    }
    var body: some View {
        HStack {
            Text(name)
            Spacer()
            Label(available ? "Ready" : "Not bundled",
                  systemImage: available ? "checkmark.circle.fill" : "xmark.circle")
                .foregroundColor(available ? .green : .orange)
                .font(.caption)
        }
    }
}

private struct ShareSheet: UIViewControllerRepresentable {
    let url: URL
    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: [url], applicationActivities: nil)
    }
    func updateUIViewController(_ vc: UIActivityViewController, context: Context) {}
}
