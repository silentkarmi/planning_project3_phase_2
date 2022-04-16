# Package for project3 Gazebo visualization

## build
```
catkin make or catkin build
```

## Run
```
roslaunch planning_project3 testing.launch
rosrun planning_project3 test_turtlebot
```

## Details
The program uses action set compose of left and right wheel velocity.  
The TurtleBot class will transform the action into cmd_vel command to control the turtlebot. 
