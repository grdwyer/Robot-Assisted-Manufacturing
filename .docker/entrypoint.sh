#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/foxy/setup.bash"
soruce "/moveit_ws/install/setup.bash"
source "/dev_ws/install/setup.bash"
exec "$@"