# Robot Assisted Manufacturing

The robot assisted manufacturing project currently focused on implant fabrication built in ROS2 foxy.

## Packages
Current packages:  
* ram_gripper_control - simulated gripper controller
* ram_interfaces - custom interfaces for ram project
* ram_motion_planning - components to plan trajectories for toolpaths
* ram_moveit_config - configuration files for moveit
* ram_support - models and URDF for the core components of the project (robots, tables, etc)
* ram_tooling_support - assistance nodes for handling (currently) toolpaths and stock material

## Running the system
This launches RVIZ, move_group node, sim iiwa controller, sim gripper controller, toolpath handler and stock handler.
Now includes some arguments that can be set (just for the ros control side at the moment)

Default (fake controller)
```
ros2 launch ram_motion_planning moveit_dev_setup.launch.py 
```

Real kuka
```
ros2 launch ram_motion_planning moveit_dev_setup.launch.py real_manipulator:=true robot_port:=30200 robot_ip:=192.170.10.2
```

This launches the moveit movegroup interface for fully constrained toolpaths (6DoF) quite simple at the moment.
```
ros2 launch ram_motion_planning toolpath_planner.launch.py  
```
When both launch files are running (wait for rviz to load), these service calls can be used
toolpath_setup will:
  * load the toolpath from the toolpath handler
  * Compute the part to tool transform
  * Plan to the approach pose
  * Plan the toolpath with the retreat pose added

toolpath execute will send the toolpath trajectory to the robot controller
```
ros2 service call /toolpath_planner/toolpath_setup std_srvs/srv/Trigger {}  
ros2 service call /toolpath_planner/toolpath_execute std_srvs/srv/Trigger {}  
```

## Docker
### Build
```bash
docker build --pull --rm -f ./.docker/Dockerfile  -t gdwyer/ram:<branch>-<platform> .
```
### Run
```bash
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --env=QT_X11_NO_MITSHM=1 \
    --workdir="/dev_ws" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    gdwyer/ram:<branch>
```
Full docker info [here](https://github.com/grdwyer/Robot-Assisted-Manufacturing/wiki/Docker)

## Hardware Support
### Iiwa
WARNING this currently only works if you run as a user and have ssh keys setup for the Kuka_iiwa_ROS2_drive repo
`/dev_ws/src/ram/.docker/add_kuka_driver.sh` 
