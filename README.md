# manipulation_demos

Launching the demo. At this moment this only works in ROS1, because of this you have to launch the demo inside the LXD container.

```
roslaunch gb_tiago_manipulation_demo pick_demo.launch
```

When the robot reachs the Home position:

```
source .bashrc_bridges
ros2 launch ros1_bridge dedicated_bridges_launch.py
```