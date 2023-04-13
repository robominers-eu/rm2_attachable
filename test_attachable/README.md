# test_attachable
Simulation mock robot with Gazebo (Ignition) Citadel and ROS2 Foxy.

## Testing

Velocity:
```
ign topic -t "cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}"

```

Test plugins separately:
```
ign topic -t /AttachableJoint -m ignition.msgs.StringMsg -p 'data: "[vehicle_blue][chassis][box2][box_body][attach]" '
 
ign topic -t /AttachableJoint -m ignition.msgs.StringMsg -p 'data: "[vehicle_blue][chassis][box2][box_body][detach]" '
 
ign topic -t /AttacherContact/contact -m ignition.msgs.StringMsg -p 'data:"[vehicle_blue][chassis][box2][box_body]"'

ign topic -t /AttacherContact/contact -m ignition.msgs.StringMsg -p 'data:"end"'

ign topic -e -t /AttacherContact/touched
```

Test ROS action:
```
ros2 action send_goal AttachableJoint test_attachable/action/AttachModel "{parent_model: "vehicle_blue", parent_link: "chassis", child_model: "box2", child_link: "box_body", attach: true}"
```