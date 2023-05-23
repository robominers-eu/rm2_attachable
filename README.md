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


Test plugins separately:
```
ign topic -t /AttachableJoint -m ignition.msgs.StringMsg -p 'data: "[rm2_sim][couplingLink][rm2_sim_mining][base_link][attach]"'
 
ign topic -t /AttachableJoint -m ignition.msgs.StringMsg -p 'data: "[rm2_sim][couplingLink][rm2_sim_mining][base_link][detach]"'
 
ign topic -t /AttacherContact/contact -m ignition.msgs.StringMsg -p 'data:"[rm2_sim][couplingLink][rm2_sim_mining][base_link]"'

ign topic -e -t /AttacherContact/touched

ign topic -t /AttacherContact/contact -m ignition.msgs.StringMsg -p 'data:"end"'
```

Test ROS action (only works if the two parts are in contact):
```
ros2 action send_goal AttachableJoint rm2_attachable/action/AttachModel "{parent_model: "rm2_sim", parent_link: "couplingLink", child_model: "rm2_sim_mining", child_link: "base_link", attach: true}"
```





