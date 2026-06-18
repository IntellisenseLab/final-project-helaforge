#!/usr/bin/env python3
"""Serve the lightweight robot web dashboard.

This process intentionally does not create a ROS node. The browser talks to ROS
through rosbridge_websocket, so the static-file server can stay tiny and avoid
one extra DDS participant on the Raspberry Pi.
"""

import argparse
import functools
import os
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

from ament_index_python.packages import get_package_share_directory


class NoCacheHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def log_message(self, fmt, *args):
        return


def default_web_root() -> str:
    return os.path.join(
        get_package_share_directory('diff_drive_robot'), 'web')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Serve the diff_drive_robot web dashboard.')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--web-root', default=default_web_root())
    parsed, _ = parser.parse_known_args(args=args)

    handler = functools.partial(NoCacheHandler, directory=parsed.web_root)
    httpd = ThreadingHTTPServer((parsed.host, parsed.port), handler)
    httpd.daemon_threads = True

    print(
        f'Web dashboard serving {parsed.web_root} on '
        f'http://{parsed.host}:{parsed.port}',
        flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        httpd.server_close()


if __name__ == '__main__':
    main()
