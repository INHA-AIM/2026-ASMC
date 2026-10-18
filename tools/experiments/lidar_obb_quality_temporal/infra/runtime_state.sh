#!/bin/bash
# Read-only record of running ROS / Docker / Main processes and sockets (no signal, no API write).
# usage: runtime_state.sh OUT.txt
OUT=$1
{
echo "captured=$(TZ=Asia/Seoul date '+%F %T %Z')"
echo "== processes (ros/roscore/rosmaster/Main/docker/python ros) =="
ps -eo pid,ppid,user,etimes,args --sort=pid | grep -E "ros|Main|docker|containerd|lidar|planner" | grep -v -E "grep|runtime_state.sh" || true
echo "== listening/udp sockets =="
ss -Htulpn 2>/dev/null || true
echo "== docker ps (read-only query) =="
if command -v docker >/dev/null 2>&1; then timeout 10 docker ps --no-trunc 2>&1 || true; else echo "docker CLI absent"; fi
echo "== ROS env in this shell =="
env | grep -E "^ROS_" || echo "(none)"
echo "== /proc/net/snmp Udp =="
grep -E "^Udp:" /proc/net/snmp; grep -E "^Udp6(Out|In)Datagrams" /proc/net/snmp6
} > "$OUT" 2>&1
echo "wrote $OUT"
