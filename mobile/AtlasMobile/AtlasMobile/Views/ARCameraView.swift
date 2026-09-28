import SwiftUI
import ARKit
import SceneKit

struct ARCameraView: UIViewRepresentable {
    let session: ARSession
    var showMesh: Bool
    var showPointCloud: Bool

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    func makeUIView(context: Context) -> ARSCNView {
        let scnView = ARSCNView()
        scnView.session = session
        scnView.automaticallyUpdatesLighting = true
        scnView.rendersCameraGrain = true
        scnView.contentMode = .scaleAspectFill
        scnView.delegate = context.coordinator
        return scnView
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        let coord = context.coordinator
        coord.showMesh = showMesh

        if showPointCloud {
            uiView.debugOptions.insert(.showFeaturePoints)
        } else {
            uiView.debugOptions.remove(.showFeaturePoints)
        }

        uiView.scene.rootNode.enumerateChildNodes { node, _ in
            if node.name == "mesh" {
                node.isHidden = !showMesh
            }
        }
    }

    final class Coordinator: NSObject, ARSCNViewDelegate {
        var showMesh = false

        func renderer(
            _ renderer: SCNSceneRenderer,
            didAdd node: SCNNode,
            for anchor: ARAnchor
        ) {
            guard showMesh, let meshAnchor = anchor as? ARMeshAnchor else { return }
            node.addChildNode(buildMeshNode(from: meshAnchor))
        }

        func renderer(
            _ renderer: SCNSceneRenderer,
            didUpdate node: SCNNode,
            for anchor: ARAnchor
        ) {
            guard let meshAnchor = anchor as? ARMeshAnchor else { return }
            node.childNodes.filter { $0.name == "mesh" }.forEach { $0.removeFromParentNode() }
            if showMesh {
                node.addChildNode(buildMeshNode(from: meshAnchor))
            }
        }

        private func buildMeshNode(from meshAnchor: ARMeshAnchor) -> SCNNode {
            let geo = meshAnchor.geometry

            let vertexSource = SCNGeometrySource(
                buffer: geo.vertices.buffer,
                vertexFormat: geo.vertices.format,
                semantic: .vertex,
                vertexCount: geo.vertices.count,
                dataOffset: geo.vertices.offset,
                dataStride: geo.vertices.stride
            )

            let faceElement = SCNGeometryElement(
                buffer: geo.faces.buffer,
                primitiveType: .triangles,
                primitiveCount: geo.faces.count,
                bytesPerIndex: geo.faces.bytesPerIndex
            )

            let scnGeo = SCNGeometry(sources: [vertexSource], elements: [faceElement])
            let material = SCNMaterial()
            material.fillMode = .lines
            material.diffuse.contents = UIColor.cyan.withAlphaComponent(0.5)
            material.isDoubleSided = true
            scnGeo.materials = [material]

            let node = SCNNode(geometry: scnGeo)
            node.name = "mesh"
            return node
        }
    }
}
