#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Generates a self-contained HTML file with an embedded Three.js point cloud viewer for a given PLY file and optionally opens it in the system browser.
import open3d as o3d
import webbrowser
import os
import sys
import numpy as np
import subprocess

def create_web_viewer(ply_file):
    """Create a web-based 3D viewer for the point cloud"""
    try:
        pcd = o3d.io.read_point_cloud(ply_file)
        if len(pcd.points) == 0:
            print("Point cloud is empty")
            return False
        
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) if pcd.has_colors() else None
        
        # Sample points for web display (limit for performance)
        max_points = min(len(points), 50000)
        if len(points) > max_points:
            indices = np.random.choice(len(points), max_points, replace=False)
            points = points[indices]
            if colors is not None:
                colors = colors[indices]
        
        # Generate JavaScript arrays
        positions_js = ','.join([f'{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}' for p in points])
        colors_js = ','.join([f'{c[0]:.3f},{c[1]:.3f},{c[2]:.3f}' for c in colors]) if colors is not None else None
        
        # Create persistent HTML file in the same directory as the PLY file
        html_file = ply_file.replace('.ply', '_viewer.html')
        
        # Add timestamp to force browser refresh
        import time
        timestamp = int(time.time())
        
        with open(html_file, 'w') as f:
            html_content = f"""<!DOCTYPE html>
<html>
<head>
    <title>3D Point Cloud Viewer</title>
    <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
    <meta http-equiv="Pragma" content="no-cache">
    <meta http-equiv="Expires" content="0">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <style>
        body {{ margin: 0; background: #000; font-family: Arial; }}
        #info {{ position: absolute; top: 10px; left: 10px; color: white; z-index: 100; }}
    </style>
</head>
<body>
    <div id="info">
        <h3>Merged Point Cloud</h3>
        <p>Points: {len(pcd.points)} (showing {len(points)})</p>
        <p>Generated: {timestamp}</p>
        <p>Use mouse to rotate, zoom, and pan</p>
    </div>
    <script>
        const scene = new THREE.Scene();
        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        const renderer = new THREE.WebGLRenderer();
        renderer.setSize(window.innerWidth, window.innerHeight);
        document.body.appendChild(renderer.domElement);
        
        const controls = new THREE.OrbitControls(camera, renderer.domElement);
        
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array([{positions_js}]);
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        
        {'const colors = new Float32Array([' + colors_js + ']);' if colors_js else ''}
        {'geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));' if colors_js else ''}
        
        const material = new THREE.PointsMaterial({{ 
            {'vertexColors: true,' if colors_js else 'color: 0x00ff00,'}
            size: 0.02 
        }});
        const points = new THREE.Points(geometry, material);
        scene.add(points);
        
        // Center camera
        geometry.computeBoundingBox();
        const center = geometry.boundingBox.getCenter(new THREE.Vector3());
        const size = geometry.boundingBox.getSize(new THREE.Vector3());
        const maxDim = Math.max(size.x, size.y, size.z);
        camera.position.copy(center);
        camera.position.z += maxDim * 2;
        controls.target.copy(center);
        
        function animate() {{
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        }}
        animate();
        
        window.addEventListener('resize', () => {{
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        }});
    </script>
</body>
</html>"""
            f.write(html_content)
        
        # Try multiple methods to open browser
        html_path = os.path.abspath(html_file)
        file_url = f'file://{html_path}'

        # Propagate display/session env vars so snap-confined browsers can open windows
        display_env = os.environ.copy()
        display_env.setdefault('DISPLAY', ':0')
        display_env.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')
        wayland = os.environ.get('_ATLAS_WAYLAND_DISPLAY') or os.environ.get('WAYLAND_DISPLAY', '')
        if wayland:
            display_env['WAYLAND_DISPLAY'] = wayland

        # In GUI mode the panel embeds the viewer directly — skip opening a browser
        if os.environ.get('ATLAS_GUI_MODE'):
            print(f"✓ 3D viewer ready: {html_path}")
            return True

        for browser, args_fn in [
            ('firefox',          lambda u: ['firefox', '--new-window', u]),
            ('chromium-browser', lambda u: ['chromium-browser', '--new-window', u]),
            ('chromium',         lambda u: ['chromium', '--new-window', u]),
            ('google-chrome',    lambda u: ['google-chrome', '--new-window', u]),
            ('xdg-open',         lambda u: ['xdg-open', u]),
        ]:
            try:
                subprocess.Popen(args_fn(file_url),
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL,
                                 env=display_env,
                                 preexec_fn=os.setpgrp)
                print(f"✓ 3D viewer opened in browser with {len(pcd.points)} points: {html_path}")
                return True
            except FileNotFoundError:
                continue

        print(f"HTML viewer created at: {html_path}")
        print("Please open this file manually in your browser")
        return False
        
    except Exception as e:
        print(f"Error creating web viewer: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) > 1:
        create_web_viewer(sys.argv[1])
    else:
        print("Usage: python web_3d_viewer.py <ply_file>")