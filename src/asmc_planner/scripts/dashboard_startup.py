"""Reserve HTTP ownership before a duplicate ROS node can evict its owner."""
import errno
import http.server
import sys


def run_on_port(port, handler, start_ros, host='0.0.0.0'):
    try:
        server = http.server.ThreadingHTTPServer((host, port), handler)
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        print('Dashboard port %d is already in use; existing server kept. '
              'Use dashboard:=false or another dashboard_port.' % port, file=sys.stderr)
        return 2
    # Also closes the reservation if ROS initialization raises.
    with server:
        return start_ros(server)
