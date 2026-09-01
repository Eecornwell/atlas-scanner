"""
Atlas Mobile — Host Enhancement Server

Receives session zips from the iOS app, runs the full enhancement pipeline
(PromptDA + StableNormal + COLMAP assembly), and serves the results back.

Usage:
    cd mobile/offline_pipeline
    python3 enhance_server.py [--host 0.0.0.0] [--port 8765] [--work-dir /tmp/atlas_jobs]

The iOS app (HostUploader.swift) connects to this server automatically when
"Host Processing" mode is selected. Set the host URL in the app to:
    http://<your-machine-ip>:8765

Find your machine IP:
    macOS:  ipconfig getifaddr en0
    Linux:  hostname -I | awk '{print $1}'

Both devices must be on the same WiFi network.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import threading
import uuid
import zipfile
from pathlib import Path

from flask import Flask, Response, jsonify, request, send_file

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Job state
# ---------------------------------------------------------------------------

_jobs: dict[str, dict] = {}  # job_id -> {status, log_lines, session_dir, result_zip}
_work_dir = Path("/tmp/atlas_jobs")


def _job(job_id: str) -> dict | None:
    return _jobs.get(job_id)


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@app.post("/upload")
def upload():
    """Receive a session zip, unpack it, start processing in background."""
    session_name = request.headers.get("X-Session-Name", "session")
    job_id = str(uuid.uuid4())[:8]

    job_dir = _work_dir / job_id
    job_dir.mkdir(parents=True, exist_ok=True)

    zip_path = job_dir / f"{session_name}"
    zip_path.write_bytes(request.data)

    # Unzip
    session_dir = job_dir / "session"
    try:
        with zipfile.ZipFile(zip_path) as zf:
            zf.extractall(session_dir)
        # Find the actual session directory inside the zip
        subdirs = [d for d in session_dir.iterdir() if d.is_dir()]
        if len(subdirs) == 1:
            session_dir = subdirs[0]
    except Exception as e:
        return jsonify({"error": f"Unzip failed: {e}"}), 400

    _jobs[job_id] = {
        "status": "queued",
        "log_lines": [],
        "session_dir": str(session_dir),
        "result_zip": None,
    }

    # Run pipeline in background thread
    thread = threading.Thread(
        target=_run_pipeline, args=(job_id, session_dir), daemon=True
    )
    thread.start()

    return jsonify({"job_id": job_id}), 200


@app.get("/status/<job_id>")
def status(job_id: str):
    """Return current job status and accumulated log lines."""
    job = _job(job_id)
    if job is None:
        return jsonify({"error": "Unknown job"}), 404
    return jsonify({
        "status": job["status"],
        "log_lines": job["log_lines"],
    })


@app.get("/download/<job_id>")
def download(job_id: str):
    """Return the enhanced results zip once processing is complete."""
    job = _job(job_id)
    if job is None:
        return jsonify({"error": "Unknown job"}), 404
    if job["status"] != "complete":
        return jsonify({"error": f"Job not complete (status: {job['status']})"}), 409
    result_zip = job.get("result_zip")
    if not result_zip or not Path(result_zip).exists():
        return jsonify({"error": "Result zip not found"}), 500
    return send_file(result_zip, mimetype="application/zip",
                     as_attachment=True,
                     download_name=Path(result_zip).name)


@app.get("/health")
def health():
    return jsonify({"status": "ok"})


# ---------------------------------------------------------------------------
# Pipeline runner
# ---------------------------------------------------------------------------

def _run_pipeline(job_id: str, session_dir: Path) -> None:
    job = _jobs[job_id]
    job["status"] = "running"

    def log(line: str) -> None:
        print(line)
        job["log_lines"].append(line)

    try:
        script = Path(__file__).parent / "enhance_session.py"
        cmd = ["python3", str(script), "--session-dir", str(session_dir)]

        log(f"Starting: {' '.join(cmd)}")
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        for line in proc.stdout:
            log(line.rstrip())
        proc.wait()

        if proc.returncode != 0:
            job["status"] = "error"
            log(f"Pipeline exited with code {proc.returncode}")
            return

        # Zip the enhanced/ and colmap/ directories for download
        result_zip = session_dir.parent / f"{session_dir.name}_enhanced.zip"
        with zipfile.ZipFile(result_zip, "w", zipfile.ZIP_DEFLATED) as zf:
            for folder in ("enhanced", "colmap"):
                src = session_dir / folder
                if src.exists():
                    for f in sorted(src.rglob("*")):
                        if f.is_file():
                            zf.write(f, f.relative_to(session_dir))

        job["result_zip"] = str(result_zip)
        job["status"] = "complete"
        log(f"✓ Complete — {result_zip.stat().st_size // 1024} KB ready for download")

    except Exception as e:
        job["status"] = "error"
        job["log_lines"].append(f"Error: {e}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Atlas Mobile host enhancement server")
    parser.add_argument("--host", default="0.0.0.0",
                        help="Bind address (default: 0.0.0.0 — all interfaces)")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--work-dir", type=Path, default=Path("/tmp/atlas_jobs"),
                        help="Directory for job working files")
    args = parser.parse_args()

    global _work_dir
    _work_dir = args.work_dir
    _work_dir.mkdir(parents=True, exist_ok=True)

    import socket
    hostname = socket.gethostname()
    try:
        local_ip = socket.gethostbyname(hostname)
    except Exception:
        local_ip = "unknown"

    print(f"Atlas Mobile Enhancement Server")
    print(f"  Listening on http://{args.host}:{args.port}")
    print(f"  Local IP (set in app): http://{local_ip}:{args.port}")
    print(f"  Work dir: {_work_dir}")
    print(f"  Health check: http://{local_ip}:{args.port}/health")
    print()

    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
