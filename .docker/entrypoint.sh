#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/galactic/setup.bash"
source "/moveit_ws/install/setup.bash"
source "/dev_ws/install/setup.bash"
exec "$@"
