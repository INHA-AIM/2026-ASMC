#!/bin/bash
# Offline python CI commands + extra nose/unittest files (py_tests.py) inside
# 'unshare --user --map-root-user --net --mount' (lo only, private binfmt_misc without WSLInterop, tmpfs /tmp),
# same isolation as the B0 audit s22_python_ns.sh. usage: py_tests_ns.sh VARIANT OUTDIR
set -u
VARIANT=$1; P=$2
RR=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239
W=$RR/repo; NSD=$W/src/lidar+sensorfusion/build/exp_$VARIANT/devel/setup.bash
mkdir -p "$P/py_home/mpl" "$P/py_home/yolo" "$P/ns_tmp"
unshare --user --map-root-user --net --mount bash -c "
ip link set lo up || exit 97
mount -t binfmt_misc binfmt_misc /proc/sys/fs/binfmt_misc || exit 96
[ -e /proc/sys/fs/binfmt_misc/WSLInterop ] && exit 96
mount -t tmpfs -o size=1m,mode=755 tmpfs /run/WSL || exit 96
mount -t tmpfs -o mode=1777,size=2g tmpfs /tmp || exit 98
unset WSL_INTEROP WSLENV
{ echo \"ns uid=\$(id -u) netns=\$(readlink /proc/self/ns/net)\"; ip -brief addr; echo \"ipv4_routes=\$(ip route | wc -l) ipv6_routes=\$(ip -6 route | wc -l)\"; echo snmp_start:; grep -E '^(Ip|Udp):' /proc/net/snmp; grep -E '^(Udp6|Ip6Out)' /proc/net/snmp6; } > '$P/ns_meta.log' 2>&1
python3 '$RR/tools/py_tests.py' '$W' '$NSD' '$P' > '$P/py_all.log' 2>&1
RC=\$?
{ echo \"runner rc=\$RC\"; echo snmp_end:; grep -E '^(Ip|Udp):' /proc/net/snmp; grep -E '^(Udp6|Ip6Out)' /proc/net/snmp6; } >> '$P/ns_meta.log' 2>&1
exit \$RC
"
echo "ns rc=$?"
grep -E '^SUMMARY' "$P/py_all.log"; grep -E 'rc=[1-9]' "$P/py_all.log" | head -20
