# Each group is a new shell. Never source a nested workspace.
source /opt/ros/noetic/setup.bash
source "${ASMC_CI_DEVEL:-devel}/setup.bash"
export ASMC_CI_BUILD="${ASMC_CI_BUILD:-build}"
export ROS_HOSTNAME=localhost
unset ROS_IP
