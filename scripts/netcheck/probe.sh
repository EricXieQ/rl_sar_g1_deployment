#!/bin/bash
# Sources ROS 2 + unitree_ros2 with DDS pinned to eth0, then runs the probe.
#   ./probe.sh baseline 60
#   ./probe.sh with-vicon 60
set -eo pipefail

# ROS setup scripts reference unbound vars, so keep `set -u` off while sourcing.
source /opt/ros/foxy/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'

# Use the system interpreter, not the eric_env venv -- ROS Foxy's Python
# packages (and numpy) live in system site-packages, and the venv lacks numpy.
exec /usr/bin/python3 "$(dirname "$0")/lowstate_probe.py" "$@"
