"""Remove dead local planning registrations before another launch claims them."""
import http.client
import ipaddress
import os
from pathlib import Path
import socket
from urllib.parse import urlsplit
import xmlrpc.client


CALLER = '/scen_runner_registry_check'


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1
        return connection


def proxy(uri):
    return xmlrpc.client.ServerProxy(uri, transport=Transport())


def local_uri(uri):
    host = urlsplit(uri).hostname
    if host in ('localhost', socket.gethostname()):
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def local_process_exists(name):
    # ROS launch passes __name to its children. A matching process is enough
    # to refuse cleanup, including a node still initializing its XML-RPC API.
    argument = ('__name:='+name.rsplit('/', 1)[-1]).encode()
    for entry in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            if argument in entry.read_bytes().split(b'\0'):
                return True
        except (FileNotFoundError, ProcessLookupError):
            pass
        except OSError:
            return True  # Cannot establish that the local process is gone.
    return False


def connection_refused(uri):
    try:
        with proxy(uri) as node:
            node.getPid(CALLER)
    except ConnectionRefusedError:
        return True
    except (OSError, xmlrpc.client.Error, http.client.HTTPException):
        pass  # Timeouts, malformed replies and network failures are ambiguous.
    return False


def node_uri(master, name):
    code, _, uri = master.lookupNode(CALLER, name)
    return uri if code == 1 else None


def prune_stale_planning_nodes(nodes, graph):
    """Never kill a node or clean unrelated names/remote registrations.

    An unreachable API alone is not sufficient: require two connection
    refusals, no matching local process, and an unchanged master URI. Pass
    the captured API to unregister calls so a replacement is not removed.
    """
    expected = {n.namespace+n.name for n in nodes}
    registered = {name for rows in graph for _, names in rows for name in names}
    candidates = sorted(expected & registered)
    if not candidates:
        return []
    removed = []
    with proxy(os.environ.get('ROS_MASTER_URI', 'http://127.0.0.1:11311')) as master:
        for name in candidates:
            uri = node_uri(master, name)
            if (not uri or not local_uri(uri) or local_process_exists(name) or
                    not connection_refused(uri)):
                continue
            code, detail, current = master.getSystemState(CALLER)
            if code != 1:
                raise RuntimeError('ROS 잔류 등록 조회 실패: '+detail)
            entries = []
            for topic, names in current[2]:
                if name in names:
                    code, _, service_uri = master.lookupService(CALLER, topic)
                    if code == 1:
                        entries.append(('unregisterService', topic, service_uri))
            for rows, method in zip(current[:2], ('unregisterPublisher', 'unregisterSubscriber')):
                entries.extend((method, topic, uri) for topic, names in rows if name in names)
            if (local_process_exists(name) or node_uri(master, name) != uri or
                    not connection_refused(uri)):
                continue
            count = 0
            for method, topic, api in entries:
                if node_uri(master, name) != uri:
                    break
                code, detail, number = getattr(master, method)(name, topic, api)
                if code != 1:
                    raise RuntimeError('ROS 잔류 등록 정리 실패: '+detail)
                count += number
            if count:
                removed.append(name)
                print(f'[판제 잔류 등록 정리] {name} · 로컬 프로세스 없음 · 등록 {count}건 해제', flush=True)
    return removed
