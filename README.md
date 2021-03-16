# manipulation_demos

Launching the demo. At this moment this only works in ROS1, because of this you have to launch the demo inside the LXD container.

```
roslaunch gb_tiago_manipulation_demo pick_simulation.launch
roslaunch gb_tiago_manipulation_demo pick_demo.launch
rosservice call /pick_gui
```