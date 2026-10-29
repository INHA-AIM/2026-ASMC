# Sourced by a container's interactive Bash; never starts ROS nodes.
# The installer supplies role-specific values in /root/.asmc-container-profile.env.
[[ ${ASMC_INTERACTIVE_ROS_PROFILE:-standalone} == off ]] && return 0
source /root/.asmc-container-profile.env
export ASMC_ROS_PROFILE=isolated
if [[ -n ${ASMC_EXTRA_SETUP:-} ]] && [[ -r $ASMC_EXTRA_SETUP ]]; then
  source "$ASMC_EXTRA_SETUP"
fi
source /opt/ros/noetic/setup.bash
if [[ ! -r $ASMC_DEVEL_SETUP ]]; then
  echo "ASMC setup missing: $ASMC_DEVEL_SETUP" >&2
  return 1
fi
source "$ASMC_DEVEL_SETUP"
export ASMC_INTERACTIVE_ROS_PROFILE=standalone
export ASMC_ROS_PROFILE=isolated
export ROS_MASTER_URI="http://127.0.0.1:$ASMC_ROS_MASTER_PORT"
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP

roscore() {
  local arg
  for arg in "$@"; do
    case "$arg" in
      -p|--port|--port=*|-p[0-9]*) command roscore "$@"; return $? ;;
    esac
  done
  command roscore -p "$ASMC_ROS_MASTER_PORT" "$@"
}

roslaunch() {
  local previous='' arg port='' requested_port='' explicit_port=false bridge=false inspect=false disabled=false
  for arg in "$@"; do
    case "$previous/$arg" in
      asmc_planner/autonomy_no_bridges_v2.launch|asmc_planner/dashboard.launch)
        port=$ASMC_PLANNING_WEB_PORT ;;
      asmc_lidar/view.launch) port=$ASMC_LIDAR_WEB_PORT ;;
      asmc_bridge/bridge.launch) bridge=true ;;
    esac
    case "$arg" in
      dashboard_port:=*) explicit_port=true; requested_port=${arg#dashboard_port:=} ;;
      dashboard:=false|dashboard:=False|dashboard:=0) disabled=true ;;
      --nodes|--files|--dump-params|--args|--find-node) inspect=true ;;
    esac
    previous=$arg
  done
  if $explicit_port; then port=$requested_port; fi
  if [[ -n $port ]] && ! $inspect && ! $disabled; then
    command python3 /root/.asmc-launch-guard.py --check-http "$port" || return $?
  fi
  if $bridge && ! $inspect; then
    command python3 /root/.asmc-launch-guard.py "$@"
  elif [[ -n $port ]] && ! $explicit_port; then
    command roslaunch "$@" "dashboard_port:=$port"
  else
    command roslaunch "$@"
  fi
}
