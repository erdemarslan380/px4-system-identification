#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import posixpath
import sys
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from pull_sdcard_logs_over_mavftp import (
    cleanup_known_holders,
    connect_mavftp,
    download_file_via_port,
    exclusive_port_lock,
    expected_size_from_entry,
    list_remote_entries,
    local_file_matches,
    wait_for_port_free,
)


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PX4 SD Card Browser</title>
  <style>
    body { font-family: sans-serif; margin: 24px; background: #f6f8fb; color: #1f2937; }
    .row { display: flex; gap: 8px; margin-bottom: 12px; flex-wrap: wrap; }
    button { padding: 8px 12px; border: 1px solid #cbd5e1; background: white; border-radius: 8px; cursor: pointer; }
    button:hover { background: #eef2ff; }
    #path { font-family: monospace; padding: 8px 12px; background: white; border: 1px solid #cbd5e1; border-radius: 8px; min-width: 320px; }
    table { width: 100%; border-collapse: collapse; background: white; border-radius: 12px; overflow: hidden; }
    th, td { padding: 10px 12px; border-bottom: 1px solid #e5e7eb; text-align: left; }
    th { background: #e2e8f0; }
    .mono { font-family: monospace; }
    .dir { color: #1d4ed8; font-weight: 600; }
    .status { margin: 12px 0; padding: 10px 12px; border-radius: 8px; background: #ecfeff; border: 1px solid #a5f3fc; white-space: pre-wrap; }
    .error { background: #fef2f2; border-color: #fecaca; color: #991b1b; }
  </style>
</head>
<body>
  <h1>PX4 SD Card Browser</h1>
  <div class="row">
    <button onclick="goHome()">Home</button>
    <button onclick="goUp()">Up</button>
    <button onclick="refreshListing()">Refresh</button>
    <div id="path"></div>
  </div>
  <div id="status" class="status">Ready.</div>
  <table>
    <thead>
      <tr>
        <th>Name</th>
        <th>Type</th>
        <th>Size</th>
        <th>Action</th>
      </tr>
    </thead>
    <tbody id="rows"></tbody>
  </table>
  <script>
    let currentPath = '/fs/microsd';
    let requestInFlight = false;

    function setStatus(message, isError=false) {
      const el = document.getElementById('status');
      el.textContent = message;
      el.className = isError ? 'status error' : 'status';
    }

    function formatBytes(bytes) {
      if (bytes === null || bytes === undefined) return '';
      const value = Number(bytes);
      if (!Number.isFinite(value) || value < 1024) return `${value} B`;
      if (value < 1024 * 1024) return `${(value / 1024).toFixed(1)} KB`;
      return `${(value / (1024 * 1024)).toFixed(1)} MB`;
    }

    function renderRows(entries) {
      const tbody = document.getElementById('rows');
      tbody.innerHTML = '';
      for (const entry of entries) {
        const tr = document.createElement('tr');

        const nameCell = document.createElement('td');
        nameCell.textContent = entry.name;
        nameCell.className = entry.is_dir ? 'dir mono' : 'mono';
        tr.appendChild(nameCell);

        const typeCell = document.createElement('td');
        typeCell.textContent = entry.is_dir ? 'directory' : 'file';
        tr.appendChild(typeCell);

        const sizeCell = document.createElement('td');
        sizeCell.textContent = entry.is_dir ? '' : formatBytes(entry.size_b);
        tr.appendChild(sizeCell);

        const actionCell = document.createElement('td');
        const button = document.createElement('button');
        if (entry.is_dir) {
          button.textContent = 'Open';
          button.onclick = () => browse(entry.path);
        } else {
          button.textContent = 'Pull To Repo';
          button.onclick = () => pullFile(entry.path);
        }
        actionCell.appendChild(button);
        tr.appendChild(actionCell);
        tbody.appendChild(tr);
      }
    }

    async function browse(path) {
      if (requestInFlight) {
        setStatus('Previous request is still running. Please wait a few seconds.', true);
        return;
      }
      requestInFlight = true;
      currentPath = path;
      document.getElementById('path').textContent = currentPath;
      setStatus(`Listing ${currentPath} ...`);
      try {
        const response = await fetch(`/api/list?path=${encodeURIComponent(path)}`);
        const payload = await response.json();
        if (!response.ok) {
          setStatus(payload.error || 'Listing failed', true);
          return;
        }
        renderRows(payload.entries);
        setStatus(`Listed ${payload.entries.length} entries in ${payload.path}`);
      } catch (error) {
        setStatus(String(error), true);
      } finally {
        requestInFlight = false;
      }
    }

    async function pullFile(path) {
      if (requestInFlight) {
        setStatus('A request is already in progress. Please wait.', true);
        return;
      }
      requestInFlight = true;
      setStatus(`Pulling ${path} ...`);
      try {
        const response = await fetch(`/api/pull?path=${encodeURIComponent(path)}`);
        const payload = await response.json();
        if (!response.ok) {
          setStatus(payload.error || 'Pull failed', true);
          return;
        }
        setStatus(`Saved to ${payload.local_path}`);
      } catch (error) {
        setStatus(String(error), true);
      } finally {
        requestInFlight = false;
      }
    }

    function goHome() {
      browse('/fs/microsd');
    }

    function goUp() {
      if (currentPath === '/fs/microsd') {
        browse(currentPath);
        return;
      }
      const parts = currentPath.split('/').filter(Boolean);
      parts.pop();
      browse('/' + parts.join('/'));
    }

    function refreshListing() {
      browse(currentPath);
    }

    browse(currentPath);
  </script>
</body>
</html>
"""


class AppState:
    def __init__(
        self,
        *,
        serial_port: str,
        baud: int,
        heartbeat_timeout: float,
        download_timeout: float,
        retries: int,
        cleanup: bool,
        port_free_timeout: float,
        remote_root: str,
        download_dir: Path,
    ) -> None:
        self.serial_port = serial_port
        self.baud = baud
        self.heartbeat_timeout = heartbeat_timeout
        self.download_timeout = download_timeout
        self.retries = retries
        self.cleanup = cleanup
        self.port_free_timeout = port_free_timeout
        self.remote_root = remote_root
        self.download_dir = download_dir


def normalize_remote_path(remote_root: str, requested_path: str) -> str:
    requested = requested_path or remote_root
    normalized = posixpath.normpath(requested)
    if not normalized.startswith("/"):
        normalized = "/" + normalized
    if normalized == "/":
        normalized = remote_root
    if not normalized.startswith(remote_root):
        raise ValueError(f"path escapes remote root: {requested_path}")
    return normalized


def local_destination_for_remote(download_dir: Path, remote_root: str, remote_path: str) -> Path:
    normalized = normalize_remote_path(remote_root, remote_path)
    relative = normalized[len(remote_root) :].lstrip("/")
    if not relative:
        raise ValueError("cannot download the root directory")
    return download_dir / relative


def list_remote_directory(state: AppState, remote_path: str) -> dict[str, Any]:
    path = normalize_remote_path(state.remote_root, remote_path)
    with exclusive_port_lock(state.serial_port):
        if state.cleanup:
            cleanup_known_holders(current_pid=os.getpid())

        if not wait_for_port_free(state.serial_port, timeout=state.port_free_timeout):
            raise RuntimeError(f"{state.serial_port} is busy")

        master, ftp = connect_mavftp(state.serial_port, state.baud, state.heartbeat_timeout)
        try:
            entries = list_remote_entries(ftp, path)
        finally:
            master.close()

    payload = []
    for entry in sorted(entries, key=lambda item: (not item.is_dir, item.name.lower())):
        payload.append(
            {
                "name": entry.name,
                "path": f"{path}/{entry.name}",
                "is_dir": bool(entry.is_dir),
                "size_b": None if entry.is_dir else expected_size_from_entry(entry),
            }
        )
    return {"path": path, "entries": payload}


def pull_remote_file(state: AppState, remote_path: str) -> dict[str, Any]:
    normalized = normalize_remote_path(state.remote_root, remote_path)
    parent = posixpath.dirname(normalized)
    basename = posixpath.basename(normalized)

    with exclusive_port_lock(state.serial_port):
        if state.cleanup:
            cleanup_known_holders(current_pid=os.getpid())

        if not wait_for_port_free(state.serial_port, timeout=state.port_free_timeout):
            raise RuntimeError(f"{state.serial_port} is busy")

        master, ftp = connect_mavftp(state.serial_port, state.baud, state.heartbeat_timeout)
        try:
            entries = list_remote_entries(ftp, parent)
            matched = next((entry for entry in entries if not entry.is_dir and entry.name == basename), None)
        finally:
            master.close()

    if matched is None:
        raise FileNotFoundError(f"remote file not found: {normalized}")

    local_path = local_destination_for_remote(state.download_dir, state.remote_root, normalized)
    expected_size = expected_size_from_entry(matched)
    existed = local_file_matches(local_path, expected_size)
    result = download_file_via_port(
        port=state.serial_port,
        baud=state.baud,
        heartbeat_timeout=state.heartbeat_timeout,
        remote_path=normalized,
        local_path=local_path,
        expected_size=expected_size,
        retries=state.retries,
        timeout=state.download_timeout,
    )
    return {
        "remote_path": normalized,
        "local_path": str(result),
        "status": "skipped" if existed else "downloaded",
    }


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "PX4SDCardBrowser/1.0"

    @property
    def state(self) -> AppState:
        return self.server.state  # type: ignore[attr-defined]

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return

    def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        encoded = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_html(self, html: str) -> None:
        encoded = html.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)

        if parsed.path == "/":
            self._send_html(HTML_PAGE)
            return

        params = parse_qs(parsed.query)
        path = params.get("path", [self.state.remote_root])[0]

        try:
            if parsed.path == "/api/list":
                self._send_json(list_remote_directory(self.state, path))
                return

            if parsed.path == "/api/pull":
                self._send_json(pull_remote_file(self.state, path))
                return

            self._send_json({"error": "not found"}, status=404)
        except Exception as exc:  # pragma: no cover - exercised by manual browser use
            self._send_json({"error": str(exc)}, status=500)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Browse CubeOrange SD-card contents over USB CDC using MAVFTP.")
    parser.add_argument("--serial-port", default="/dev/ttyACM0", help="USB CDC serial port")
    parser.add_argument("--baud", type=int, default=57600, help="MAVLink baud rate")
    parser.add_argument("--http-port", type=int, default=8765, help="Local HTTP port for the browser UI")
    parser.add_argument("--heartbeat-timeout", type=float, default=10.0, help="Heartbeat wait timeout")
    parser.add_argument("--download-timeout", type=float, default=30.0, help="Per-file MAVFTP timeout")
    parser.add_argument("--retries", type=int, default=1, help="Retry count per file")
    parser.add_argument("--cleanup-known-holders", action="store_true", help="Kill old jMAVSim/MAVLink helper processes first")
    parser.add_argument("--port-free-timeout", type=float, default=5.0, help="Seconds to wait for the CDC port to become free")
    parser.add_argument("--remote-root", default="/fs/microsd", help="Remote root directory")
    parser.add_argument(
        "--download-dir",
        type=Path,
        default=Path("~/px4-system-identification/hitl_runs/browser_downloads").expanduser(),
        help="Local directory where pulled files will be written",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    state = AppState(
        serial_port=args.serial_port,
        baud=args.baud,
        heartbeat_timeout=args.heartbeat_timeout,
        download_timeout=args.download_timeout,
        retries=args.retries,
        cleanup=args.cleanup_known_holders,
        port_free_timeout=args.port_free_timeout,
        remote_root=normalize_remote_path(args.remote_root, args.remote_root),
        download_dir=args.download_dir.resolve(),
    )
    state.download_dir.mkdir(parents=True, exist_ok=True)

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), RequestHandler)
    server.state = state  # type: ignore[attr-defined]
    print(f"PX4 SD browser listening on http://127.0.0.1:{args.http_port}/")
    print(f"Downloads will be saved under {state.download_dir}")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
