# Project3 - Phase 2
It simulates the differential drive robot in gazebo
--------------------------------------------------------------------------------------------------
>> How to run the ROS CODE:
	- Go inside catkin_ws
 	- catkin build
 	- roslaunch planning_project3 demo.launch
 	(launches the python and the ros code)
 	
>> After the roslaunch command:
 	- Input Start and Goal values
 	- Spinning Cursor means that the path is being calculated
 	- The to-be-path is show on opencv, after this it will ask
 	- Start the robot in gazebo (y/n): y
 	- And the robot will start moving 
 	
 	
>> Note:
X Y coordinates are in metres in float, thetha is in degrees
Origin is (0,0) in Gazebo Simulator
Min value is (-5.0, -5.0) of coordinate
Max value is (5.0, 5.0) of coordinate
Because its an open-loop there is slight errors when following way-points

    
>> Video Output of the program is included: dif_drive_gazebo_simulation.mkv
>> This project has been done in a GROUP:
    - Kartikeya Mishra (UID: 118106755)
    - Chang-Hong Chen (UID: 117397857)

