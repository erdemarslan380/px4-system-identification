from __future__ import annotations

import argparse
import functools
import http.server
import socketserver
import threading
import time
import webbrowser
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


class ReusableTCPServer(socketserver.TCPServer):
    allow_reuse_address = True


def main() -> int:
    parser = argparse.ArgumentParser(description="Serve local SITL docs/review HTML files and optionally open one in a browser.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--open-path",
        default="",
        help="Path relative to the repo root, e.g. docs/sitl_validation/three_model/review/index.html",
    )
    args = parser.parse_args()

    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(REPO_ROOT))
    with ReusableTCPServer((args.host, args.port), handler) as httpd:
        httpd_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        httpd_thread.start()
        base_url = f"http://{args.host}:{args.port}"
        print(f"Serving {REPO_ROOT} at {base_url}")

        if args.open_path:
            open_path = args.open_path.lstrip("/")
            url = f"{base_url}/{open_path}"
            print(f"Opening {url}")
            webbrowser.open(url)

        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("Shutting down docs server.")
            httpd.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
