#!/usr/bin/env python3
"""UDP / isolation audit for one namespace run. SKELETON (designed, not executed in the design wave).

usage: udp_audit.py --meta ns_meta.log [--strace strace.txt] [--planner-port 7510] [--forbidden-ports 11311,11312,11400]

Inputs
  ns_meta.log : written by ns_exec_b0.sh; contains 'snmp_start:' / 'snmp_end:' blocks with the Ip/Udp lines of
                /proc/net/snmp (IPv4 only) followed by Udp6*/Ip6Out* lines of /proc/net/snmp6 (IPv6; the previous
                harness missed these, verify_runtime.txt D6; e03 [U] shows Udp6OutDatagrams moves while
                Udp.OutDatagrams does not).
  strace.txt  : strace -f -yy -e trace=execve,clone,fork,vfork,socket,bind,connect,sendto,sendmsg,sendmmsg,write,writev
                write/writev are included because a write() on a connected UDP socket is invisible otherwise
                (verify_runtime.txt D6; e03 [S] shows 'write(3<UDP:[127.0.0.1:x->127.0.0.1:27998]>' with the new set).
Output: one JSON object on stdout. Exit 0 always (verdicts are data; 90_collect.py decides).
"""
import argparse
import json
import os
import re
from collections import defaultdict

LINE = re.compile(r'^(\d+)\s+(.*)$')
CALL = re.compile(r'^(\w+)\((.*)\)\s+=\s+(-?\d+|\?)(.*)$')
UNFIN = re.compile(r'^(\w+)\((.*)\s+<unfinished \.\.\.>$')
RESUM = re.compile(r'^<\.\.\. (\w+) resumed>(.*)\)\s+=\s+(-?\d+|\?)(.*)$')
UDPFD = re.compile(r'^(\d+)<(UDP|UDPv6):\[([^\]]*)\]>')
PORT = re.compile(r'sin6?_port=htons\((\d+)\)')
DEST4 = re.compile(r'sin_addr=inet_addr\("([^"]+)"\)')
DEST6 = re.compile(r'inet_pton\(AF_INET6,\s*"([^"]+)"')
ERRN = re.compile(r'\s(E[A-Z0-9]+)\s')
SENDS = ('sendto', 'sendmsg', 'sendmmsg', 'write', 'writev')


def loopback(host):
    return host.startswith('127.') or host in ('::1', '0.0.0.0', '::') or host.startswith('::ffff:127.')


def snmp_blocks(text):
    out = {}
    for tag in ('snmp_start', 'snmp_end'):
        m = re.search(tag + r':\n((?:(?:Ip|Udp):.*\n|(?:Udp6|Ip6Out)\S*\s+\d+\n)+)', text)
        if not m:
            out[tag] = None
            continue
        d, lines = {}, m.group(1).splitlines()
        v4 = [l for l in lines if l.startswith(('Ip:', 'Udp:'))]
        for i in range(0, len(v4) - 1, 2):
            k, v = v4[i].split(), v4[i + 1].split()
            for a, b in zip(k[1:], v[1:]):
                d[k[0][:-1] + '.' + a] = int(b)
        for l in lines:
            if l.startswith(('Udp6', 'Ip6Out')):
                a, b = l.split()[:2]
                d[a] = int(b)
        out[tag] = d
    keys = ('Udp.OutDatagrams', 'Udp.NoPorts', 'Udp.InDatagrams', 'Ip.OutRequests', 'Ip.OutNoRoutes',
            'Udp6OutDatagrams', 'Udp6NoPorts', 'Udp6InDatagrams', 'Ip6OutRequests', 'Ip6OutNoRoutes')
    s, e = out['snmp_start'], out['snmp_end']
    res = {'start': {k: s.get(k) for k in keys} if s else None, 'end': {k: e.get(k) for k in keys} if e else None}
    res['delta'] = {k: e[k] - s[k] for k in keys if s and e and k in s and k in e} if (s and e) else None
    res['ipv6_recorded'] = bool(s and 'Udp6OutDatagrams' in s)
    return res


def parse_strace(path):
    pending, events = {}, []
    with open(path, errors='replace') as f:
        for raw in f:
            m = LINE.match(raw.rstrip('\n'))
            if not m:
                continue
            tid, rest = int(m.group(1)), m.group(2)
            if rest.startswith(('+++', '---')):
                continue
            u = UNFIN.match(rest)
            if u:
                pending[tid] = (u.group(1), u.group(2))
                continue
            r = RESUM.match(rest)
            if r:
                name, a0 = pending.pop(tid, (r.group(1), ''))
                events.append((tid, name, a0 + r.group(2), r.group(3), r.group(4)))
                continue
            c = CALL.match(rest)
            if c:
                events.append((tid, c.group(1), c.group(2), c.group(3), c.group(4)))
    return events


def audit_strace(path, planner_port, forbidden):
    events = parse_strace(path)
    prog = {}
    per = defaultdict(lambda: {'udp_socket_calls': 0, 'udp_bind': [], 'udp_connect': [], 'udp_send_ok': 0,
                               'udp_send_err': 0, 'udp_send_by_call': defaultdict(int),
                               'udp_send_dest': defaultdict(int), 'udp_send_errno': defaultdict(int),
                               'udp_send_nonloopback_ok': 0, 'planner_port_sends': 0})
    forbidden_refs, nonloop_inet, suspicious_exec = [], [], []
    for tid, name, args, ret, tail in events:
        if name == 'execve':
            mm = re.match(r'"([^"]+)"', args)
            if mm:
                exe = mm.group(1)
                if ret == '0':
                    prog[tid] = os.path.basename(exe)
                if exe.lower().endswith('.exe') or exe.startswith(('/mnt/', '/init')):
                    suspicious_exec.append({'tid': tid, 'exe': exe, 'ret': ret})
            continue
        if name in ('clone', 'fork', 'vfork', 'clone3'):
            if ret not in ('?',) and int(ret) > 0:
                prog[int(ret)] = prog.get(tid, 'unknown(pid %d)' % tid)
            continue
        pname = prog.get(tid, 'unknown')
        # forbidden ports / non-loopback inet anywhere in socket-address arguments (any protocol)
        for pm in PORT.finditer(args):
            if int(pm.group(1)) in forbidden:
                forbidden_refs.append({'prog': pname, 'call': name, 'args': args[:200], 'ret': ret})
        for host in DEST4.findall(args) + DEST6.findall(args):
            if not loopback(host) and name in ('connect', 'sendto', 'sendmsg', 'sendmmsg', 'bind'):
                nonloop_inet.append({'prog': pname, 'call': name, 'host': host, 'ret': ret, 'tail': tail.strip()[:60]})
        p = per[pname]
        if name == 'socket':
            if re.search(r'AF_INET6?,\s*SOCK_DGRAM', args):
                p['udp_socket_calls'] += 1
            continue
        fdm = UDPFD.match(args)
        if not fdm:
            continue
        if name == 'bind':
            p['udp_bind'].append({'fd_info': fdm.group(3), 'args': args[:160], 'ret': ret})
        elif name == 'connect':
            p['udp_connect'].append({'args': args[:200], 'ret': ret, 'tail': tail.strip()[:80]})
        elif name in SENDS:
            pm = PORT.search(args)
            h = (DEST4.findall(args) + DEST6.findall(args) + [None])[0]
            dest = ('%s:%s' % (h, pm.group(1))) if (pm and h) else 'connected:' + fdm.group(3)
            p['udp_send_by_call'][name] += 1
            p['udp_send_dest'][dest] += 1
            if dest.endswith(':%d' % planner_port) or dest.endswith('->127.0.0.1:%d' % planner_port):
                p['planner_port_sends'] += 1
            if ret != '?' and int(ret) >= 0:
                p['udp_send_ok'] += 1
                host = dest.split(':')[0] if not dest.startswith('connected:') else ''
                if host and not loopback(host):
                    p['udp_send_nonloopback_ok'] += 1
            else:
                p['udp_send_err'] += 1
                e = ERRN.search(' ' + tail + ' ')
                p['udp_send_errno'][e.group(1) if e else 'unknown'] += 1
    tot = defaultdict(int)
    for v in per.values():
        for k in ('udp_socket_calls', 'udp_send_ok', 'udp_send_err', 'udp_send_nonloopback_ok', 'planner_port_sends'):
            tot[k] += v[k]
        tot['udp_connect'] += len(v['udp_connect'])
        tot['udp_bind'] += len(v['udp_bind'])
    touched = {k: v for k, v in per.items() if v['udp_socket_calls'] or v['udp_bind'] or v['udp_connect']
               or v['udp_send_ok'] or v['udp_send_err']}
    return {'strace_file': path, 'events_parsed': len(events), 'programs_seen': sorted(set(prog.values())),
            'per_program_udp': touched, 'totals': dict(tot), 'forbidden_port_refs': forbidden_refs,
            'nonloopback_inet': nonloop_inet, 'suspicious_exec': suspicious_exec,
            'main_udp_socket_calls': per['main']['udp_socket_calls'] if 'main' in per else 0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--meta', required=True)
    ap.add_argument('--strace')
    ap.add_argument('--planner-port', type=int, default=7510)
    ap.add_argument('--forbidden-ports', default='11311,11312,11400')
    a = ap.parse_args()
    forbidden = {int(x) for x in a.forbidden_ports.split(',') if x}
    meta = open(a.meta, errors='replace').read()
    out = {'snmp': snmp_blocks(meta),
           'ns_routes_zero': bool(re.search(r'ipv4_routes=0 ipv6_routes=0', meta)),
           'ns_lo_only': None, 'wslinterop_absent_in_ns': 'WSLInterop' not in (re.search(
               r'binfmt_misc entries in ns: ([^\n]*)', meta) or [None, 'WSLInterop'])[1]}
    m = re.search(r'interfaces:\n((?:.*\n){1,6}?)ipv4_routes', meta)
    if m:
        ifaces = [l.split()[0] for l in m.group(1).splitlines() if l.strip()]
        out['ns_lo_only'] = ifaces == ['lo']
    if a.strace and os.path.exists(a.strace):
        out['strace'] = audit_strace(a.strace, a.planner_port, forbidden)
    d = (out['snmp'] or {}).get('delta') or {}
    st = out.get('strace', {})
    out['verdict_inputs'] = {
        'ipv4_udp_out': d.get('Udp.OutDatagrams'), 'ipv6_udp_out': d.get('Udp6OutDatagrams'),
        'ipv4_out_noroutes': d.get('Ip.OutNoRoutes'), 'ipv6_out_noroutes': d.get('Ip6OutNoRoutes'),
        'strace_udp_send_ok': (st.get('totals') or {}).get('udp_send_ok'),
        'strace_udp_send_err': (st.get('totals') or {}).get('udp_send_err'),
        'strace_udp_connect': (st.get('totals') or {}).get('udp_connect'),
        'strace_planner_port_sends': (st.get('totals') or {}).get('planner_port_sends'),
        'main_udp_socket_calls': st.get('main_udp_socket_calls'),
        'forbidden_port_refs': len(st.get('forbidden_port_refs', [])) if st else None,
        'nonloopback_inet': len(st.get('nonloopback_inet', [])) if st else None,
        'suspicious_exec': len(st.get('suspicious_exec', [])) if st else None}
    print(json.dumps(out, indent=1, default=lambda o: dict(o)))


if __name__ == '__main__':
    main()
