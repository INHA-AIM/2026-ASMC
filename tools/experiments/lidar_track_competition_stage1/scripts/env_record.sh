#!/bin/bash
# Record build/runtime environment (read-only).
OUT="$1"
{
  echo "== date"; date -Iseconds
  echo "== uname"; uname -a
  echo "== lsb"; lsb_release -a 2>/dev/null
  echo "== cpu"; lscpu | egrep "Model name|^CPU\(s\)|Thread|Core|Socket|MHz"
  echo "== mem"; free -m
  echo "== gpu"; (nvidia-smi -L 2>/dev/null || echo "no nvidia-smi")
  echo "== gcc"; gcc --version | head -1
  echo "== cmake"; cmake --version | head -1
  echo "== ros"; ls /opt/ros; (source /opt/ros/noetic/setup.bash; rosversion -d; echo ROS_DISTRO=$ROS_DISTRO)
  echo "== pcl"; dpkg -l | egrep "libpcl-dev|libeigen3-dev|libopencv-dev |libtbb-dev" | awk "{print \$2, \$3}"
  echo "== python"; python3 --version; python3 -c "import numpy; print('numpy', numpy.__version__)"; python3 -c "import scipy; print('scipy', scipy.__version__)" 2>&1 | tail -1
  echo "== disk"; df -h /home
  echo "== load"; uptime
  echo "== top cpu procs"; ps aux --sort=-%cpu | head -8
} > "$OUT" 2>&1
echo done
