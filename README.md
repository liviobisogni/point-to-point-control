# Point-to-point control

A ROS package designed to enable a turtle robot to move from a starting point to a user-defined goal pose using a proportional controller.

<p align="center">
	<a href="#prerequisite">Prerequisite</a>
	<span> • </span>
	<a href="#compile">Compile</a>
	<span> • </span>
	<a href="#execute">Execute</a>
	<span> • </span>
	<a href="#use">Use</a>
	<span> • </span>
	<a href="#screenshots">Screenshots</a>
</p>

## <a id="prerequisite"></a>Prerequisite

* [ROS](http://wiki.ros.org/ROS/Installation) - An open-source, meta-operating system for your robots. The repository has been tested using ROS Kinetic.

## <a id="compile"></a>How to Compile
1. Move this package folder (`point-to-point-control`) to the `src` directory of your ROS workspace, for example `~/catkin_ws/src/`.

2. Open a terminal window and navigate to your ROS workspace directory, for example:

	```bash
	cd ~/catkin_ws/
	```
3. Build the package using the `catkin_make` command:

	```bash
	catkin_make
	```
This will compile the package and generate the necessary files for running the ROS nodes.


## <a id="execute"></a>How to Execute
1. Open a terminal window.
2. Navigate to your ROS workspace directory.
3. Launch the package using the following command:
	```bash
	roslaunch point_to_point_control point_to_point_control.launch
	```
This will launch the ROS nodes required to run the package. 


## <a id="use"></a>How to Use

1. Open a terminal window and pass a goal pose (`goalPose_X`, `goalPose_Y`) using the following command:

	```bash
	rostopic pub /turtle1/PositionCommand geometry_msgs/Pose2D '{x: goalPose_X, y: goalPose_Y}' -1
	```
where `goalPose_X` and `goalPose_Y` are values between 0 and 11.
2. Once a goal pose is passed, the turtle will start moving towards it using a proportional controller.
3. Once the turtle reaches the goal pose, it will stop and wait for another desired pose to be passed.
4. Any poses that are out of bounds are properly treated. The program prints various types of information on the terminal, and you can exit the program at any time by pressing the `ESC` key.


## <a id="screenshots"></a>Screenshots

* Initial position:
<p align="center" width="100%">
    <img width="61.8%" src="img/p1.png"> 
</p>

* Turtle is moving towards the first goal pose (selected by the user and passed via the terminal window):
<p align="center" width="100%">
    <img width="61.8%" src="img/p2.png"> 
</p>

* Fifth goal pose reached:
<p align="center" width="100%">
    <img width="61.8%" src="img/p3.png"> 
</p>
