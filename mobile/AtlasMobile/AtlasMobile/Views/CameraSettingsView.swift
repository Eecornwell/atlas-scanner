import SwiftUI

struct CameraSettingsView: View {
    @State private var config = MultiCameraConfig.loadFromDeviceOrBundle()
    @State private var showAddCamera = false
    @State private var editingIndex: Int?
    @State private var saveMessage: String?
    @State private var connectionStatus: String?
    @State private var isTesting = false
    @State private var connectionLog: [String] = []

    var body: some View {
        List {
            Section("Configured Cameras") {
                if config.cameras.isEmpty {
                    Text("No Insta360 cameras configured")
                        .foregroundColor(.secondary)
                } else {
                    ForEach(Array(config.cameras.enumerated()), id: \.element.id) { index, cam in
                        CameraRow(camera: cam)
                            .contentShape(Rectangle())
                            .onTapGesture { editingIndex = index }
                    }
                    .onDelete(perform: deleteCamera)
                }
            }

            Section {
                Button {
                    showAddCamera = true
                } label: {
                    Label("Add Insta360 Camera", systemImage: "plus.circle")
                }
            }

            if !config.cameras.isEmpty {
                Section("Connection Test") {
                    Button {
                        Task { await testConnection() }
                    } label: {
                        if isTesting {
                            HStack {
                                ProgressView()
                                    .padding(.trailing, 4)
                                Text("Connecting…")
                            }
                        } else {
                            Label("Test Connection", systemImage: "antenna.radiowaves.left.and.right")
                        }
                    }
                    .disabled(isTesting)

                    if let status = connectionStatus {
                        Text(status)
                            .font(.subheadline)
                            .foregroundColor(status.contains("connected") ? .green : .red)
                    }

                    if !connectionLog.isEmpty {
                        DisclosureGroup("Connection Log") {
                            ForEach(
                                Array(connectionLog.enumerated()),
                                id: \.offset
                            ) { _, line in
                                Text(line)
                                    .font(.system(.caption2, design: .monospaced))
                                    .foregroundColor(.secondary)
                            }
                        }
                        Button {
                            UIPasteboard.general.string = connectionLog
                                .joined(separator: "\n")
                        } label: {
                            Label("Copy Log to Clipboard", systemImage: "doc.on.doc")
                        }
                    }
                }
            }

            if let msg = saveMessage {
                Section {
                    Label(msg, systemImage: "checkmark.circle.fill")
                        .foregroundColor(.green)
                }
            }
        }
        .navigationTitle("Camera Settings")
        .sheet(isPresented: $showAddCamera) {
            CameraFormView(title: "Add Camera", buttonLabel: "Add") { newCamera in
                var cameras = config.cameras
                cameras.append(newCamera)
                config = MultiCameraConfig(cameras: cameras, iphone: config.iphone)
                save()
            }
        }
        .sheet(item: $editingIndex) { index in
            let cam = config.cameras[index]
            CameraFormView(
                title: "Edit Camera",
                buttonLabel: "Save",
                existing: cam
            ) { updated in
                var cameras = config.cameras
                cameras[index] = updated
                config = MultiCameraConfig(cameras: cameras, iphone: config.iphone)
                save()
            }
        }
    }

    private func deleteCamera(at offsets: IndexSet) {
        var cameras = config.cameras
        cameras.remove(atOffsets: offsets)
        config = MultiCameraConfig(cameras: cameras, iphone: config.iphone)
        save()
    }

    private func testConnection() async {
        isTesting = true
        connectionLog = []
        connectionStatus = "Searching (10s timeout)…"
        let manager = Insta360CaptureManager(config: config)
        manager.onLogMessage = { msg in
            Task { @MainActor in
                self.connectionLog.append(msg)
            }
        }
        await manager.discoverAndConnect()
        let count = manager.connectedCameras.count
        let total = config.cameras.count
        if count == total {
            connectionStatus = "\(count) camera\(count == 1 ? "" : "s") connected"
        } else if count > 0 {
            connectionStatus = "\(count)/\(total) cameras connected"
        } else {
            connectionStatus = "No cameras found.\n"
                + "1. Power on the Insta360\n"
                + "2. Connect iPhone to camera WiFi\n"
                + "3. Verify serial number matches"
        }
        await manager.disconnect()
        isTesting = false
    }

    private func save() {
        do {
            try config.saveToDocuments()
            saveMessage = "Saved — restart session to apply"
        } catch {
            saveMessage = nil
        }
    }
}

extension Int: @retroactive Identifiable {
    public var id: Int { self }
}

private struct CameraRow: View {
    let camera: CameraConfig

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack {
                Text(camera.id)
                    .font(.headline)
                Spacer()
                Image(systemName: "chevron.right")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            Text("\(camera.model) — \(camera.serial)")
                .font(.caption)
                .foregroundColor(.secondary)
            let e = camera.extrinsic
            Text("XYZ: \(e.x, specifier: "%.3f"), \(e.y, specifier: "%.3f"), \(e.z, specifier: "%.3f") m")
                .font(.system(.caption2, design: .monospaced))
                .foregroundColor(.secondary)
            Text("RPY: \(e.roll, specifier: "%.1f")° \(e.pitch, specifier: "%.1f")° \(e.yaw, specifier: "%.1f")°")
                .font(.system(.caption2, design: .monospaced))
                .foregroundColor(.secondary)
        }
        .padding(.vertical, 2)
    }
}

private struct CameraFormView: View {
    @Environment(\.dismiss) var dismiss

    let title: String
    let buttonLabel: String
    var onSave: (CameraConfig) -> Void

    @State private var id: String
    @State private var model: String
    @State private var serial: String
    @State private var forwardM: Double
    @State private var leftM: Double
    @State private var upM: Double
    @State private var rollDeg: Double
    @State private var pitchDeg: Double
    @State private var yawDeg: Double

    init(title: String, buttonLabel: String, existing: CameraConfig? = nil,
         onSave: @escaping (CameraConfig) -> Void) {
        self.title = title
        self.buttonLabel = buttonLabel
        self.onSave = onSave
        _id = State(initialValue: existing?.id ?? "insta360_01")
        _model = State(initialValue: existing?.model ?? "X5")
        _serial = State(initialValue: existing?.serial ?? "")
        _forwardM = State(initialValue: existing?.extrinsic.x ?? 0.0)
        _leftM = State(initialValue: existing?.extrinsic.y ?? 0.0)
        _upM = State(initialValue: existing?.extrinsic.z ?? 0.0)
        _rollDeg = State(initialValue: existing?.extrinsic.roll ?? 0.0)
        _pitchDeg = State(initialValue: existing?.extrinsic.pitch ?? 0.0)
        _yawDeg = State(initialValue: existing?.extrinsic.yaw ?? 0.0)
    }

    var body: some View {
        NavigationStack {
            Form {
                Section("Camera Info") {
                    TextField("ID (e.g. insta360_01)", text: $id)
                    TextField("Model (e.g. X5)", text: $model)
                    TextField("Serial Number", text: $serial)
                }

                Section("Mount Position (meters)") {
                    NumberField("Forward", value: $forwardM)
                    NumberField("Left", value: $leftM)
                    NumberField("Up", value: $upM)
                }

                Section("Mount Rotation (degrees)") {
                    NumberField("Roll", value: $rollDeg)
                    NumberField("Pitch", value: $pitchDeg)
                    NumberField("Yaw", value: $yawDeg)
                }
            }
            .navigationTitle(title)
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") { dismiss() }
                }
                ToolbarItem(placement: .confirmationAction) {
                    Button(buttonLabel) {
                        let cam = CameraConfig(
                            id: id,
                            model: model,
                            serial: serial,
                            extrinsic: RigidTransform(
                                roll: rollDeg, pitch: pitchDeg, yaw: yawDeg,
                                x: forwardM, y: leftM, z: upM
                            ),
                            mask: nil,
                            faceCount: 8,
                            tileFov: 65.0
                        )
                        onSave(cam)
                        dismiss()
                    }
                    .disabled(id.isEmpty)
                }
            }
        }
    }
}

private struct NumberField: View {
    let label: String
    @Binding var value: Double

    init(_ label: String, value: Binding<Double>) {
        self.label = label
        self._value = value
    }

    var body: some View {
        HStack {
            Text(label)
            Spacer()
            Button {
                value = -value
            } label: {
                Image(systemName: "plus.forwardslash.minus")
                    .font(.caption)
                    .foregroundColor(.accentColor)
            }
            .buttonStyle(.bordered)
            .controlSize(.small)
            TextField("0.0", value: $value, format: .number)
                .keyboardType(.decimalPad)
                .multilineTextAlignment(.trailing)
                .frame(width: 100)
        }
    }
}
