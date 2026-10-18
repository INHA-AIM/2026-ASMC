#!/bin/bash
# Run a batch of ROS-dependent jobs inside ONE fresh unprivileged user+net+pid+mount namespace with a private
# roscore (hardening copied from the B0 audit ns_exec_b0.sh): lo up only (0 routes), private binfmt_misc
# (WSLInterop absent -> abort), tmpfs /run/WSL and /tmp, WSL/ROS env unset, private master port in
# [20000,29999] (never 11311/11312/11400), /proc/net/snmp + snmp6 before/after, full teardown.
# usage: ns_batch.sh NAME SETUP_BASH JOBFILE PARALLEL TIMEOUT_S [STRACE(0|1)]
#   JOBFILE: one shell command per line, executed with xargs -P PARALLEL (each must write its own outputs).
set -u
NAME=$1; SETUP=$2; JOBS=$3; PAR=$4; TMO=$5; STRACE=${6:-0}
RR=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239
D=$RR/logs/ns/$NAME; rm -rf "$D"; mkdir -p "$D" "$RR/logs/ns/ros_home/$NAME/log"
while :; do PORT=$(( 20000 + RANDOM % 10000 )); case $PORT in 11311|11312|11400) continue;; esac
  ss -Htuln 2>/dev/null | awk '{print $5}' | grep -q ":$PORT\$" || break; done
RH=$RR/logs/ns/ros_home/$NAME
INNER=$D/inner.sh
cat > "$INNER" <<EOF
#!/bin/bash
ip link set lo up || { echo "lo up failed"; exit 97; }
mount -t binfmt_misc binfmt_misc /proc/sys/fs/binfmt_misc || { echo "binfmt_misc private mount failed"; exit 96; }
[ -e /proc/sys/fs/binfmt_misc/WSLInterop ] && { echo "WSLInterop visible inside ns -> abort"; exit 96; }
mount -t tmpfs -o size=1m,mode=755 tmpfs /run/WSL || { echo "tmpfs /run/WSL failed"; exit 96; }
mount -t tmpfs -o size=2g,mode=1777 tmpfs /tmp || { echo "tmpfs /tmp failed"; exit 96; }
unset WSL_INTEROP WSLENV WSL_DISTRO_NAME WSL2_GUI_APPS_ENABLED ROS_HOSTNAME ROS_IP ROS_MASTER_URI ROS_PACKAGE_PATH PYTHONPATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH
export HOME=/home/autonav USER=autonav LOGNAME=autonav LANG=C.UTF-8 LC_ALL=C.UTF-8 TERM=dumb PYTHONDONTWRITEBYTECODE=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export ROS_HOME=$RH ROS_LOG_DIR=$RH/log
set +u; source /opt/ros/noetic/setup.bash; source '$SETUP'
export ROS_MASTER_URI=http://127.0.0.1:$PORT ROS_IP=127.0.0.1
snap() { echo "\$1:"; grep -E '^(Ip|Udp):' /proc/net/snmp; grep -E '^(Udp6|Ip6Out)' /proc/net/snmp6; }
{
echo "name=$NAME start=\$(date '+%F %T.%N %Z')"
echo "netns=\$(readlink /proc/self/ns/net) pidns=\$(readlink /proc/self/ns/pid) userns=\$(readlink /proc/self/ns/user) mntns=\$(readlink /proc/self/ns/mnt) uid_in_ns=\$(id -u)"
echo "interfaces:"; ip -brief addr; echo "ipv4_routes=\$(ip route | wc -l) ipv6_routes=\$(ip -6 route | wc -l)"
echo "binfmt_misc entries in ns: \$(ls /proc/sys/fs/binfmt_misc | tr '\n' ' ')  /run/WSL: [\$(ls -A /run/WSL | tr '\n' ' ')]"
/mnt/c/Windows/System32/cmd.exe /c exit 0 > /dev/null 2>&1; echo "interop_probe_rc=\$? (non-zero = Windows interop blocked)"
echo "ROS_MASTER_URI=\$ROS_MASTER_URI setup=$SETUP private_master_port=$PORT"
snap snmp_start
} > '$D/ns_meta.log' 2>&1
setsid roscore -p $PORT > '$D/roscore.log' 2>&1 &
for i in \$(seq 1 150); do rosparam get /run_id >/dev/null 2>&1 && break; sleep 0.1; done
echo "roscore up run_id=\$(rosparam get /run_id 2>&1)" >> '$D/ns_meta.log'
T0=\$(date +%s.%N)
if [ "$STRACE" = 1 ]; then
  timeout -k 20 $TMO strace -f --seccomp-bpf -qq -e signal=none -yy -s 48 -e trace=execve,clone,fork,vfork,socket,bind,connect,sendto,sendmsg,sendmmsg,write,writev -o '$D/strace.txt' \
    bash -c "xargs -P $PAR -I{} -d '\n' bash -c '{}' < '$JOBS'" > '$D/out.log' 2>&1
else
  timeout -k 20 $TMO bash -c "xargs -P $PAR -I{} -d '\n' bash -c '{}' < '$JOBS'" > '$D/out.log' 2>&1
fi
RC=\$?
echo "\$RC" > '$D/rc.txt'
{
echo "rc=\$RC wall_s=\$(awk -v a=\$T0 -v b=\$(date +%s.%N) 'BEGIN{printf "%.1f", b-a}')"
snap snmp_end
echo "udp sockets before teardown:"; ss -Huanp 2>/dev/null | head -20
echo "ns processes before teardown:"; ps -eo pid,ppid,comm | head -40
for p in \$(ps -eo pid= | tr -d ' '); do [ "\$p" != 1 ] && [ "\$p" != \$\$ ] && kill -TERM \$p 2>/dev/null; done; sleep 1
for p in \$(ps -eo pid= | tr -d ' '); do [ "\$p" != 1 ] && [ "\$p" != \$\$ ] && kill -KILL \$p 2>/dev/null; done
echo "end=\$(date '+%F %T.%N %Z')"
} >> '$D/ns_meta.log' 2>&1
exit \$RC
EOF
unshare --user --map-root-user --net --pid --fork --mount-proc bash "$INNER"
RC=$?
python3 -I - "$D/ns_meta.log" > "$D/udp_snmp.json" <<'PY'
import json, re, sys
t = open(sys.argv[1]).read()
def block(tag):
    m = re.search(tag + r':\n((?:(?:Ip|Udp):.*\n|(?:Udp6|Ip6Out)\S*\s+\d+\n)+)', t)
    if not m: return None
    d, lines = {}, m.group(1).splitlines()
    v4 = [l for l in lines if l.startswith(('Ip:', 'Udp:'))]
    for i in range(0, len(v4) - 1, 2):
        k, v = v4[i].split(), v4[i + 1].split()
        for a, b in zip(k[1:], v[1:]): d[k[0][:-1] + '.' + a] = int(b)
    for l in lines:
        if l.startswith(('Udp6', 'Ip6Out')):
            a, b = l.split()[:2]; d[a] = int(b)
    return d
s, e = block('snmp_start'), block('snmp_end')
keys = ('Udp.OutDatagrams', 'Udp.InDatagrams', 'Udp.NoPorts', 'Udp6OutDatagrams', 'Udp6InDatagrams', 'Udp6NoPorts')
delta = {k: e[k] - s[k] for k in keys if s and e and k in s and k in e}
print(json.dumps({"start": s and {k: s.get(k) for k in keys}, "end": e and {k: e.get(k) for k in keys}, "delta": delta,
                  "udp_out_total": (delta.get('Udp.OutDatagrams', 0) + delta.get('Udp6OutDatagrams', 0)) if delta else None,
                  "routes": re.findall(r'ipv4_routes=(\d+) ipv6_routes=(\d+)', t)}))
PY
echo "$NAME rc=$RC udp=$(cat "$D/udp_snmp.json")"
exit $RC
