#!/usr/bin/env bash

branch=$(git branch --show-current)

echo -e "Starting up RAM control container for branch $branch\nWARNING this container will be deleted once left, use ctrl-p then ctrl-q to detach and leave running and docker cp to take files if necessary"

docker run -it \
    --rm \
    --workdir="/dev_ws" \
    --net=host \
    --cap-add SYS_NICE \
    --name ram_control \
    gdwyer/ram:$branch-amd64 \
    ros2 launch ram_motion_planning ram_control.launch.py real_gripper:=true us_cutter:=true
