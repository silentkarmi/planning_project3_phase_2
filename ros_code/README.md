# Package for project3 Gazebo visualization

## Install submodule
```
git submodule update --init --recursive
```
## build
```
cd catkin_ws
catkin make or catkin build
source devel/setup.zsh
```

## Run
```
roslaunch planning_project3 testing.launch
rosrun planning_project3 test_turtlebot
```

## Details
The program uses action sets compose of left and right wheel velocity.  
The TurtleBot class will transform the action into cmd_vel command to control the turtlebot. 
