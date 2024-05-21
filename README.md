# Enabling Safe and Stable Drone Delivery
### Henry Buron

Payload stabilization.

### Notes
1. ROS2 Humble

Comment out `source /opt/ros/iron/setup.bash` from bashrc

```
colcon clean workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Note: issue with `ros2 node list` not returning anything, even if nodes are active. However, ros2 topic list works, and the nodes are actually there.

2. SSH into Jetson Orin Nano

```
ssh -X henry@192.168.18.108
```

3. Build only payload package
```
colcon build --packages-select payload
```