#!/usr/bin/env bash

branch=$(git branch --show-current)

echo -e "Starting up RAM control container for branch $branch\nWARNING this container will be deleted once left, use ctrl-p then ctrl-q to detach and leave running and docker cp to take files if necessary"

docker run -dit \
    --rm \
    --workdir="/dev_ws" \
    --net=host \
    --cap-add SYS_NICE \
    --name ram_control \
    gdwyer/ram:$branch-amd64

mkdir /tmp/ram_control_startup
cd /tmp/ram_control_startup

git clone git@github.com:grdwyer/Kuka_iiwa_ROS2_Driver.git
git clone -b foxy git@github.com:grdwyer/iiwa_fri_description.git

docker cp Kuka_iiwa_ROS2_Driver ram_control:/dev_ws/src/
docker cp iiwa_fri_description ram_control:/dev_ws/src/

cd /tmp
rm -rf /tmp/ram_control_startup

docker attach ram_control