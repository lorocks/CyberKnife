# CyberKnife
The following submission contains the SolidWorks part and assembly for CyberKnife, a ROS2 package to move the CyberKnife autonomously and through user-input.
There are multiple launch and node run files in this package.

## Plugins
### cyberknife Package
Implementation of CyberKnife in ROS2

### odometry Package
Odometry plugin built to be used in the package

## Launch Files
There are three launch files:-
	- display.launch.py:- Used to launch RViz
	- cyberknife_pos.launch.py:- Used to launch CyberKnife using the position controller
	- cyberknife_vel.launch.py:- Used to launch CyberKnife using the velocity controller
	
## Nodes
	- draw_circle_pos.py:- Draws a circle for inverse kinematic validation using position controller
	- draw_circle_vel.py:- Draws a circle for inverse kinematic validation using velocity controller
	- gravity_stabalize.py:- Node that stabalizes the CyberKnife at initial configuration negating the influence of gravity
	- pid_joint_state.py:- Node that uses PI controller to move towards target point and remain at constant position coordinates, but varying Euler angles.
	- teleop_gui_slider.py:- Used to teleoperate the CyberKnife using GUI sliders as a position controller scheme
	- teleop_gui_slider_velocity.py:- Used to teleoperate the CyberKnife using GUI sliders as a velocity controller scheme
	- teleop_keyboard: Used to teleoperate the CyberKnife using the keyboard as a velocity controller scheme

## Worlds
A world file which contains the added Robonaut2 asset is used to simulate a patient for the CyberKnife.

## Config
In the config directory, there are two .yaml files to setup the controller__manager for ros2_control. One uses position control and the other velocity control.
There are two extra files, Jacopianp.txt and TransformationMatrix.txt which are pickled SymPy symbolic expression which is loaded by the package to reduce code execution time and not have to compute the matrices on every run.


# Running Information
All position control related nodes need to be run after launching using,
```bash
	ros2 launch cyberknife cyberknife_pos.launch.py
```
Similarly all velocity control related nodes need to be run after launching using,
```bash
	ros2 launch cyberknife cyberknife_vel.launch.py
```
Nodes can be run using,
```bash
	ros2 run cyberknife <node executable name>
```


# Video Links
## Inverse Kinematic Validation Video Link
https://www.youtube.com/watch?v=FfMCb4wMvGk

## Gravity Stabilization Video Link
https://www.youtube.com/watch?v=1uS9T0m9D7o

## PI Controller Video Link
https://www.youtube.com/watch?v=j_8HjirhPN0
