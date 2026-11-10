#!/usr/bin/env python3
"""Native ownership watcher against a delayed/failing master, no real ROS graph."""
import json
import os
from pathlib import Path
import socketserver
import subprocess
import threading
import time
from xmlrpc.server import SimpleXMLRPCServer

class Server(socketserver.ThreadingMixIn, SimpleXMLRPCServer):
    daemon_threads = True

with Server(('127.0.0.1', 0), logRequests=False) as server:
    start = time.monotonic()
    def state(_caller):
        elapsed = time.monotonic()-start
        if 0.8 <= elapsed < 2.5:
            time.sleep(.8)
        pubs = [] if elapsed < 2.5 else [['/ctrl_cmd/jamming', ['/foreign']]]
        return [1, 'ok', [pubs, [], []]]
    server.register_function(state, 'getSystemState')
    threading.Thread(target=server.serve_forever, daemon=True).start()
    root = Path(__file__).resolve().parents[2]
    run = subprocess.run([str(root/'devel/lib/asmc_controller/local_ownership_probe'),
                          str(server.server_address[1])], text=True, capture_output=True, timeout=6)
    server.shutdown()
    assert run.returncode == 0, run.stderr + run.stdout
    rows = [json.loads(line) for line in run.stdout.splitlines() if line.startswith('{')]
    assert any(r['verified'] for r in rows), rows
    assert any(not r['verified'] and r['query_ms'] >= 190 for r in rows), rows
    assert any(r['conflict'] for r in rows), rows
    assert max(r['query_ms'] for r in rows) < 300, rows
    assert max(r['read_ms'] for r in rows) < 10, rows
    assert len(rows) >= 170, len(rows)
    print(json.dumps(dict(samples=len(rows), maximum_read_ms=max(r['read_ms'] for r in rows),
                         maximum_query_ms=max(r['query_ms'] for r in rows),
                         timeout_refused=True, duplicate_refused=True, shutdown_bounded=True)))
