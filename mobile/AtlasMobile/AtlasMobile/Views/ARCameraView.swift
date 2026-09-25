import SwiftUI
import ARKit
import SceneKit

struct ARCameraView: UIViewRepresentable {
    let session: ARSession

    func makeUIView(context: Context) -> ARSCNView {
        let scnView = ARSCNView()
        scnView.session = session
        scnView.automaticallyUpdatesLighting = true
        scnView.rendersCameraGrain = true
        scnView.contentMode = .scaleAspectFill
        return scnView
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}
}
