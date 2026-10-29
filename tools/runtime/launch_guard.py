#!/usr/bin/env python3
"""Check console ownership or hold the physical lease for a manual full bridge.

No simulator RPC, scene change, node termination, packet relay or collection.
This wrapper only gates an explicitly requested roslaunch invocation.
"""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys


UDP_PORTS = (9099, 9798, 9092, 9291, 9293, 9295, 9297, 9299, 9301, 9303)


def check_http_port(port):
    port = int(port)
    if not 1024 <= port <= 65535:
        raise RuntimeError('invalid console port: ' + str(port))
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind(('0.0.0.0', port))
        except OSError as error:
            raise RuntimeError('Console port %d is already owned. Existing nodes were '
                               'left untouched; choose another dashboard_port.' % port) from error


def check_udp_ports():
    sockets = []
    try:
        for port in UDP_PORTS:
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sockets.append(receiver)
            try:
                receiver.bind(('0.0.0.0', port))
            except OSError as error:
                raise RuntimeError(
                    'MORAI UDP %d is already owned. Stop the active bridge in its '
                    'original terminal before switching containers; nothing was stopped.' % port
                ) from error
    finally:
        for receiver in sockets:
            receiver.close()


def run(arguments):
    if not arguments or 'bridge.launch' not in arguments or 'asmc_bridge' not in arguments:
        raise RuntimeError('only the canonical asmc_bridge bridge.launch is supported')
    directory = os.environ.get('ASMC_VALIDATION_LEASE_DIR')
    if not directory or not Path(directory).is_dir():
        raise RuntimeError('shared physical lease directory is missing; bridge not started')
    sys.path.insert(0, '/root/ws/tools/scen_runner')
    from scen_runner.runtime.validation_lease import ValidationLease
    leases = []
    child = None
    previous_handlers = {}

    def forward(number, _frame):
        if child is not None and child.poll() is None:
            try:
                os.killpg(child.pid, number)
            except ProcessLookupError:
                pass
        else:
            raise KeyboardInterrupt

    try:
        for endpoint in ('host.docker.internal', '127.0.0.1'):
            lease = ValidationLease(endpoint, 7789, directory)
            lease.acquire()
            leases.append(lease)
        check_udp_ports()
        for number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[number] = signal.signal(number, forward)
        print('ASMC bridge owner: %s | %s | physical lease held' % (
            os.environ.get('ASMC_RUNTIME_LABEL', 'this container'),
            os.environ.get('ROS_MASTER_URI', 'unset')), flush=True)
        child = subprocess.Popen(['roslaunch'] + arguments, start_new_session=True)
        result = child.wait()
        return result if result >= 0 else 128 - result
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(10)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.wait(5)
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)
        for lease in reversed(leases):
            lease.release()


if __name__ == '__main__':
    try:
        if len(sys.argv) == 3 and sys.argv[1] == '--check-http':
            check_http_port(sys.argv[2])
        else:
            sys.exit(run(sys.argv[1:]))
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as error:
        print('ASMC launch not started: ' + str(error), file=sys.stderr)
        sys.exit(2)
