#!/usr/bin/env python3
import os
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


if __name__ == "__main__":
    directory = Path(__file__).resolve().parent
    os.chdir(directory)
    server = ThreadingHTTPServer(("127.0.0.1", 8093), SimpleHTTPRequestHandler)
    print("KairoSpatial visualizer: http://127.0.0.1:8093/spatial_visualizer.html")
    server.serve_forever()
