import SwiftUI
import UIKit

/// The Insta360 live preview: the render view of the
/// INSCameraSessionPlayer in Insta360LivePreview, as on the SDK sample's
/// take-picture page.
struct Insta360PreviewTile: View {
    @ObservedObject var preview: Insta360LivePreview

    var body: some View {
        ZStack(alignment: .bottomLeading) {
            Insta360PlayerView(renderView: preview.renderView)
                .frame(width: 150, height: 100)
                .clipShape(RoundedRectangle(cornerRadius: 8))
                .overlay(
                    RoundedRectangle(cornerRadius: 8)
                        .stroke(.white, lineWidth: 2)
                )
                .shadow(radius: 4)

            Text(preview.isRunning ? "360 live" : "360 not live")
                .font(.system(size: 9, weight: .bold))
                .foregroundColor(.white)
                .padding(.horizontal, 3)
                .padding(.vertical, 1)
                .background(.black.opacity(0.6))
                .cornerRadius(3)
                .padding(3)
        }
    }
}

/// Shows the player's render view. The player has one render view, so it
/// moves into the container shown last.
private struct Insta360PlayerView: UIViewRepresentable {
    /// Nil until Insta360LivePreview.start() creates the player.
    let renderView: UIView?

    func makeUIView(context: Context) -> PreviewContainer {
        let container = PreviewContainer()
        container.backgroundColor = .black
        container.clipsToBounds = true
        container.show(renderView)
        return container
    }

    func updateUIView(_ container: PreviewContainer, context: Context) {
        container.show(renderView)
    }
}

/// Holds the render view and sizes it to fill the tile.
private final class PreviewContainer: UIView {
    func show(_ view: UIView?) {
        guard let view, view.superview !== self else { return }
        view.removeFromSuperview()
        addSubview(view)
        setNeedsLayout()
    }

    override func layoutSubviews() {
        super.layoutSubviews()
        for view in subviews {
            view.frame = bounds
        }
    }
}
