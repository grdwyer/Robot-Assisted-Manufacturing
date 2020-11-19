# Robot Assisted Manufacturing
The robot assisted manufacturing project currently focused on implant fabrication built in ROS2 foxy.

## Packages
Current packages:  
  * ram_support - models and URDF for the core components of the project (robots, tables, etc)


## Docker 
All packages within this repo should be all building within a docker container.

### Build
This will hopefully be working on both AMD64 and ARM64 platforms
```
docker build --pull --rm -f ./.docker/Dockerfile  -t gdwyer/ram:<branch>-<platform> .
```
This will eventually be combined to a super image using docker manifests.   
```
docker manifest create \
gdwyer/ram:<branch>-<platform> \
--amend gdwyer/ram:<branch>-<platform> \
--amend gdwyer/ram:<branch>-<platform>

docker manifest push gdwyer/ram:<branch>
```

### Running

#### Nvidia
If you have an nvida GPU and want GUIs to run properly you'll need nvidia docker 2 following instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

My approach (2.3 from the [ROS guide](http://wiki.ros.org/docker/Tutorials/GUI))
```
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --env=QT_X11_NO_MITSHM=1 \
    --workdir="/dev_ws/src" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --gpus 'all,"capabilities=graphics,compute,utility"'\
    gdwyer/ram:<branch>
```
#### Normal Running
My approach (2.3 from the [ROS guide](http://wiki.ros.org/docker/Tutorials/GUI))
```
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --env=QT_X11_NO_MITSHM=1 \
    --workdir="/dev_ws/src" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    gdwyer/ram:<branch>
```