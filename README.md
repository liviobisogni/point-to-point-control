# __Point-to-point control__

### _Author_: Livio Bisogni
###### __&copy; 2021 Turtley & Turtles Ltd.__
___
I'll take you where no one's ever turtled before.

## Prerequisites

* [ROS](http://wiki.ros.org/ROS/Installation) - An open-source, meta-operating system for your robots. Repository tested only under ROS Kinetic, though.

## How to compile
1. Move this folder (`point_to_point_control`) in `~/catkin_ws/src` (or wherever thy ROS workspace is).
2. Launch a terminal window and navigate to the aforementioned ROS workspace, e.g.,

	```
	$ cd ~/catkin_ws/
	```
3. Build the package:

	```
	$ catkin_make
	```

## How to execute
Open the terminal and type:

```
$ roslaunch point_to_point_control point_to_point_control.launch
```

## How to use
	$ rostopic pub /turtle1/PositionCommand geometry_msgs/Pose2D '{x: goalPose_X, y: goalPose_Y}' -1
	```

![](img/p1.png)

![](img/p2.png)

![](img/p3.png)