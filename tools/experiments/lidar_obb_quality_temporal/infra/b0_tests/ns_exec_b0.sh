#!/bin/bash
# Run ONE ROS-dependent command inside a FRESH unprivileged user+net+pid+mount namespace (G4).
# Adapted from analysis/tests/skeletons/ns_exec_b0.sh and logs/g3/s40_rostest_ns.sh.
# usage: ns_exec_b0.sh NAME SETUP_BASH CWD TIMEOUT_S STRACE(0|1) 'command string'
# out:   $RT/NAME/{ns_meta.log,out.log,rc.txt,strace.txt,udp_audit.json,host_before_after.txt}
# roslaunch itself starts rosmaster on the private port of ROS_MASTER_URI, so rosmaster/rosout/Main/
# feeder/recorder are all inside the traced process tree.
source "$(dirname "$(readlink -f "$0")")/common_b0.sh"
NAME=$1; SETUP=$2; CWD=$3; TMO=$4; STRACE=$5; CMD=$6
D=$RT/$NAME; mkdir -p "$D" "$RT/gen" "$RT/ros_home/$NAME/log"
INNER=$RT/gen/$NAME.inner.sh
TRACE_SET=execve,clone,fork,vfork,socket,bind,connect,sendto,sendmsg,sendmmsg,write,writev
cat > "$INNER" <<EOF
#!/bin/bash
# ---- isolation (pid 1 of the new pid ns) ----
ip link set lo up || { echo "lo up failed"; exit 97; }
mount -t binfmt_misc binfmt_misc /proc/sys/fs/binfmt_misc || { echo "binfmt_misc private mount failed"; exit 96; }
[ -e /proc/sys/fs/binfmt_misc/WSLInterop ] && { echo "WSLInterop visible inside ns -> abort"; exit 96; }
mount -t tmpfs -o size=1m,mode=755 tmpfs /run/WSL || { echo "tmpfs /run/WSL failed"; exit 96; }
mount -t tmpfs -o size=512m,mode=1777 tmpfs /tmp || { echo "tmpfs /tmp failed"; exit 96; }
unset WSL_INTEROP WSLENV WSL_DISTRO_NAME WSL2_GUI_APPS_ENABLED ROS_HOSTNAME ROS_IP ROS_MASTER_URI ROS_PACKAGE_PATH PYTHONPATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH
export HOME=/home/autonav USER=autonav LOGNAME=autonav LANG=C.UTF-8 LC_ALL=C.UTF-8 TERM=dumb PYTHONDONTWRITEBYTECODE=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export ROS_HOME=$RT/ros_home/$NAME ROS_LOG_DIR=$RT/ros_home/$NAME/log
set +u; source /opt/ros/noetic/setup.bash; source '$SETUP'
export ROS_MASTER_URI=http://127.0.0.1:$PORT ROS_IP=127.0.0.1
snap() { echo "\$1:"; grep -E '^(Ip|Udp):' /proc/net/snmp; grep -E '^(Udp6|Ip6Out)' /proc/net/snmp6; }
{
echo "name=$NAME start=\$(date '+%F %T.%N %Z')"
echo "netns=\$(readlink /proc/self/ns/net) pidns=\$(readlink /proc/self/ns/pid) userns=\$(readlink /proc/self/ns/user) mntns=\$(readlink /proc/self/ns/mnt) uid_in_ns=\$(id -u)"
echo "interfaces:"; ip -brief addr; echo "ipv4_routes=\$(ip route | wc -l) ipv6_routes=\$(ip -6 route | wc -l)"
echo "binfmt_misc entries in ns: \$(ls /proc/sys/fs/binfmt_misc | tr '\n' ' ')  /run/WSL: [\$(ls -A /run/WSL | tr '\n' ' ')]"
echo "WSL env: \$(env | grep -c '^WSL') vars"
/mnt/c/Windows/System32/cmd.exe /c exit 0 > /dev/null 2>&1; echo "interop_probe_rc=\$? (non-zero = Windows interop blocked)"
echo "ROS_MASTER_URI=\$ROS_MASTER_URI ROS_IP=\$ROS_IP ROS_HOME=\$ROS_HOME setup=$SETUP"
echo "private_master_port=$PORT"
echo "Main package: \$(rospack find Main 2>&1)  main binary: \$(catkin_find --libexec Main main 2>&1 | head -1)"
snap snmp_start
} > '$D/ns_meta.log' 2>&1
cd '$CWD'
T0=\$(date +%s.%N)
if [ "$STRACE" = 1 ]; then
  timeout -k 20 $TMO strace -f --seccomp-bpf -qq -e signal=none -yy -s 48 -e trace=$TRACE_SET -o '$D/strace.txt' \
      bash -c $(printf '%q' "$CMD") > '$D/out.log' 2>&1
else
  timeout -k 20 $TMO bash -c $(printf '%q' "$CMD") > '$D/out.log' 2>&1
fi
RC=\$?
echo "\$RC" > '$D/rc.txt'
{
echo "rc=\$RC wall_s=\$(awk -v a=\$T0 -v b=\$(date +%s.%N) 'BEGIN{printf "%.1f", b-a}')"
snap snmp_end
echo "udp sockets before teardown:"; ss -Huanp 2>/dev/null | head -20
echo "listening tcp before teardown:"; ss -Htlnp 2>/dev/null | head -20
echo "ns processes before teardown (pid 1 = this shell):"; ps -eo pid,ppid,comm,args | cut -c1-200
for p in \$(ps -eo pid= | tr -d ' '); do [ "\$p" != 1 ] && [ "\$p" != \$\$ ] && kill -TERM \$p 2>/dev/null; done; sleep 1
for p in \$(ps -eo pid= | tr -d ' '); do [ "\$p" != 1 ] && [ "\$p" != \$\$ ] && kill -KILL \$p 2>/dev/null; done
echo "ns processes after teardown: \$(ps -eo pid= | wc -l) (pid 1 + ps)"
echo "end=\$(date '+%F %T.%N %Z')"
} >> '$D/ns_meta.log' 2>&1
exit \$RC
EOF
{
echo "--- host before $NAME $(date '+%F %T')"; ros_ps; echo "listeners(11311|11312|11400|7510|$PORT):"; host_listeners
} > "$D/host_before_after.txt" 2>&1
unshare --user --map-root-user --net --pid --fork --mount-proc bash "$INNER"
RC=$?
sleep 0.5
{
echo "--- host after $NAME (must be empty) $(date '+%F %T')"; ros_ps; echo "listeners:"; host_listeners
echo "(end)"
} >> "$D/host_before_after.txt" 2>&1
STRACE_ARG=(); [ "$STRACE" = 1 ] && STRACE_ARG=(--strace "$D/strace.txt")
python3 -I "$SK/udp_audit.py" --meta "$D/ns_meta.log" "${STRACE_ARG[@]}" --planner-port 7510 \
  --forbidden-ports 11311,11312,11400 > "$D/udp_audit.json" 2> "$D/udp_audit.err"
echo "$NAME rc=$RC"
exit $RC
