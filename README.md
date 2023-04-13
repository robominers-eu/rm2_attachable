# rm2_attachable
Simulation of UPM robot with Gazebo (Ignition) Citadel and ROS2 Foxy.

## Install
Create workspace and clone this repo:

```
source /opt/ros/foxy/setup.bash
mkdir -p ~/rm2_ws/src
cd ~/rm2_ws/src
git clone git@github.com:robominers-eu/rm2_attachable.git
cd ..
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy 
colcon build --symlink-install
source install/setup.bash
```

