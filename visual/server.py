#!/usr/bin/env python3
# TEACHING FILE: Localhost-bound Python stdlib ThreadingHTTPServer serves static generated visualization. It neither runs source compilation nor executes tests and should remain bound to loopback.
import os
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


if __name__ == "__main__":
    directory = Path(__file__).resolve().parent
    os.chdir(directory)
    server = ThreadingHTTPServer(("127.0.0.1", 8093), SimpleHTTPRequestHandler)
    print("KairoSpatial visualizer: http://127.0.0.1:8093/spatial_visualizer.html")
    server.serve_forever()
