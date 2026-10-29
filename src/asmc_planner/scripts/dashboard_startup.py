"""Reserve HTTP ownership before a duplicate ROS node can evict its owner."""
import errno
import http.server
import sys
import json
from pathlib import Path
import time
import urllib.request
import urllib.error


def release_idle_trial_monitor(port):
    """Reclaim only this worktree's ended observer, never active/foreign UI."""
    workspace=str(Path(__file__).resolve().parents[3])
    try:
        base='http://127.0.0.1:%d'%port
        with urllib.request.urlopen(base+'/api/monitor-health',timeout=.2) as r:info=json.load(r)
        if info.get('protocol')!='asmc-trial-monitor-v1' or info.get('workspace')!=workspace:return False
        request=urllib.request.Request(base+'/api/monitor/release-idle',method='POST',
                                       headers={'X-ASMC-Workspace':workspace})
        with urllib.request.urlopen(request,timeout=.2) as r:return bool(json.load(r).get('released'))
    except (OSError,ValueError) as error:
        if isinstance(error,urllib.error.HTTPError):error.close()
        return False


def run_on_port(port, handler, start_ros, host='0.0.0.0'):
    try:
        server = http.server.ThreadingHTTPServer((host, port), handler)
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        if release_idle_trial_monitor(port):
            # Reserve before ROS registration, as in the ordinary startup.
            for _ in range(20):
                try:
                    server=http.server.ThreadingHTTPServer((host,port),handler)
                except OSError as retry:
                    if retry.errno!=errno.EADDRINUSE:raise
                    time.sleep(.05)
                    continue
                with server:return start_ros(server)
        print('Dashboard port %d is already in use; existing server kept. '
              'Use dashboard:=false or another dashboard_port.' % port, file=sys.stderr)
        return 2
    # Also closes the reservation if ROS initialization raises.
    with server:
        return start_ros(server)
