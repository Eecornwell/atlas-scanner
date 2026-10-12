import SwiftUI
import UIKit

struct MaskDrawingView: View {
    let erpImage: UIImage
    let cameraId: String
    @Environment(\.dismiss) private var dismiss

    @State private var strokes: [MaskStroke] = []
    @State private var currentPoints: [CGPoint] = []
    @State private var brushRadius: CGFloat = 25
    @State private var isErasing = false

    private struct MaskStroke {
        let points: [CGPoint]
        let radius: CGFloat
        let isEraser: Bool
    }

    var body: some View {
        VStack(spacing: 0) {
            GeometryReader { geo in
                let fitSize = fittedSize(
                    image: erpImage.size, in: geo.size
                )
                let offsetX = (geo.size.width - fitSize.width) / 2
                let offsetY = (geo.size.height - fitSize.height) / 2

                ZStack {
                    Image(uiImage: erpImage)
                        .resizable()
                        .scaledToFit()
                        .frame(
                            width: fitSize.width,
                            height: fitSize.height
                        )

                    Canvas { ctx, size in
                        drawAllStrokes(
                            in: &ctx,
                            fitSize: fitSize
                        )
                    }
                    .frame(
                        width: fitSize.width,
                        height: fitSize.height
                    )
                    .contentShape(Rectangle())
                    .gesture(
                        DragGesture(minimumDistance: 0)
                            .onChanged { v in
                                let pt = v.location
                                guard pt.x >= 0,
                                      pt.x <= fitSize.width,
                                      pt.y >= 0,
                                      pt.y <= fitSize.height
                                else { return }
                                let norm = CGPoint(
                                    x: pt.x / fitSize.width,
                                    y: pt.y / fitSize.height
                                )
                                currentPoints.append(norm)
                            }
                            .onEnded { _ in
                                if !currentPoints.isEmpty {
                                    let normRadius =
                                        brushRadius / fitSize.width
                                    strokes.append(MaskStroke(
                                        points: currentPoints,
                                        radius: normRadius,
                                        isEraser: isErasing
                                    ))
                                    currentPoints = []
                                }
                            }
                    )
                }
                .position(
                    x: geo.size.width / 2,
                    y: geo.size.height / 2
                )
            }

            controls
        }
        .navigationTitle("Mask — \(cameraId)")
        .navigationBarTitleDisplayMode(.inline)
        .onAppear { loadExistingMask() }
    }

    private var controls: some View {
        VStack(spacing: 8) {
            HStack {
                Text("Brush \(Int(brushRadius))")
                    .font(.caption)
                    .frame(width: 64, alignment: .leading)
                Slider(
                    value: $brushRadius,
                    in: 5.0...80.0,
                    step: 1.0
                )
            }
            HStack(spacing: 12) {
                Button {
                    isErasing.toggle()
                } label: {
                    Label(
                        isErasing ? "Eraser" : "Paint",
                        systemImage: isErasing
                            ? "eraser" : "paintbrush.fill"
                    )
                    .font(.caption)
                }
                .buttonStyle(.bordered)
                .tint(isErasing ? .orange : .red)

                Button {
                    if !strokes.isEmpty {
                        strokes.removeLast()
                    }
                } label: {
                    Label("Undo", systemImage: "arrow.uturn.backward")
                        .font(.caption)
                }
                .buttonStyle(.bordered)
                .disabled(strokes.isEmpty)

                Button {
                    strokes.removeAll()
                } label: {
                    Label("Clear", systemImage: "trash")
                        .font(.caption)
                }
                .buttonStyle(.bordered)
                .foregroundColor(.red)
                .disabled(strokes.isEmpty)

                Spacer()

                Button {
                    saveMask()
                    dismiss()
                } label: {
                    Label("Save", systemImage: "checkmark")
                        .font(.caption)
                }
                .buttonStyle(.borderedProminent)
            }
        }
        .padding(.horizontal)
        .padding(.vertical, 8)
        .background(.ultraThinMaterial)
    }

    // MARK: - Drawing

    private func drawAllStrokes(
        in ctx: inout GraphicsContext,
        fitSize: CGSize
    ) {
        for stroke in strokes {
            drawStroke(stroke, in: &ctx, fitSize: fitSize)
        }
        if !currentPoints.isEmpty {
            let normRadius = brushRadius / fitSize.width
            let active = MaskStroke(
                points: currentPoints,
                radius: normRadius,
                isEraser: isErasing
            )
            drawStroke(active, in: &ctx, fitSize: fitSize)
        }
    }

    private func drawStroke(
        _ stroke: MaskStroke,
        in ctx: inout GraphicsContext,
        fitSize: CGSize
    ) {
        let color: Color = stroke.isEraser
            ? .blue.opacity(0.3)
            : .red.opacity(0.4)
        let r = stroke.radius * fitSize.width
        for normPt in stroke.points {
            let px = normPt.x * fitSize.width
            let py = normPt.y * fitSize.height
            ctx.fill(
                Path(ellipseIn: CGRect(
                    x: px - r, y: py - r,
                    width: r * 2, height: r * 2
                )),
                with: .color(color)
            )
        }
    }

    // MARK: - Save / Load

    private func saveMask() {
        let maskW = Int(erpImage.size.width)
        let maskH = Int(erpImage.size.height)
        print("[Mask] saveMask: \(strokes.count) strokes, erpSize=\(maskW)x\(maskH)")

        guard maskW > 0, maskH > 0 else {
            print("[Mask] ERROR: erpImage size is zero")
            return
        }

        let renderer = UIGraphicsImageRenderer(
            size: CGSize(width: maskW, height: maskH)
        )
        let mask = renderer.image { _ in
            UIColor.black.setFill()
            UIRectFill(CGRect(
                x: 0, y: 0, width: maskW, height: maskH
            ))

            for stroke in strokes {
                if stroke.isEraser {
                    UIColor.black.setFill()
                } else {
                    UIColor.white.setFill()
                }
                let r = stroke.radius * CGFloat(maskW)
                for normPt in stroke.points {
                    let x = normPt.x * CGFloat(maskW)
                    let y = normPt.y * CGFloat(maskH)
                    UIBezierPath(ovalIn: CGRect(
                        x: x - r, y: y - r,
                        width: r * 2, height: r * 2
                    )).fill()
                }
            }
        }

        guard let dir = Self.maskDir else {
            print("[Mask] ERROR: could not resolve Documents directory")
            return
        }

        do {
            try FileManager.default.createDirectory(
                at: dir, withIntermediateDirectories: true
            )
        } catch {
            print("[Mask] ERROR creating dir: \(error)")
        }

        let url = dir.appendingPathComponent("\(cameraId).png")
        guard let data = mask.pngData() else {
            print("[Mask] ERROR: pngData() returned nil")
            return
        }

        do {
            try data.write(to: url)
            let exists = FileManager.default.fileExists(
                atPath: url.path
            )
            let size = (try? FileManager.default.attributesOfItem(
                atPath: url.path
            )[.size] as? Int) ?? 0
            print("[Mask] SAVED \(maskW)x\(maskH) → \(url.path) exists=\(exists) bytes=\(size)")
        } catch {
            print("[Mask] ERROR writing: \(error)")
        }
    }

    private func loadExistingMask() {
        guard let dir = Self.maskDir else {
            print("[Mask] loadExisting: no Documents dir")
            return
        }
        let url = dir.appendingPathComponent("\(cameraId).png")
        let exists = FileManager.default.fileExists(atPath: url.path)
        print("[Mask] loadExisting: \(url.path) exists=\(exists)")
    }

    // MARK: - Helpers

    private static let maskDir: URL? = {
        FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first?.appendingPathComponent("atlas_sessions/masks")
    }()

    private func fittedSize(
        image: CGSize, in container: CGSize
    ) -> CGSize {
        let scale = min(
            container.width / image.width,
            container.height / image.height
        )
        return CGSize(
            width: image.width * scale,
            height: image.height * scale
        )
    }
}
