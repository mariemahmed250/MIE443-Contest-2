# MIE443-Contest-2

Contest 2 - Finding Objects of Interest in an Environment

How the code runs:
1. Open four terminator windows on Ubuntu inside the catkin_ws folder from the submission package.
2. Run the following command in each of the four windows:
	source devel/setup.bash
3. Run the following command in a window:
	catkin_make
4. In the first window, run the following command to launch gazebo: 
	roslaunch mie443_contest2 turtlebot_world.launch world:=practice
5. In the second window, run the following command to run AMCL localization and obstacle avoidance algorithm: 
	roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/turtlebot/catkin_ws/src/mie443_contest2/maps/map_practice.yaml
6. In the third window, run the following command to see the visualization of the algorithm: 
	roslaunch turtlebot_rviz_launchers view_navigation.launch
7. Before running the code in the fourth window, localize the robot in RViz by giving accurate 2D Pose Estimate and several 2D Nav Goals around the map.
7. In the fourth window, run the following command to run the contest2.cpp file: 
	rosrun mie443_contest2 contest2
8. Once the simulation of map exploration is finished, Results.txt file is set to be saved in the following directory:
	/home/turtlebot/catkin_ws/src/mie443_contest2/Results.txt

***
Notes:
- Change the map file names in steps 4 and 5 accordingly if tested in a different environment.
- Change the saving location of the Results.txt file in contest2.cpp accordingly if the above address in step 8 does not correspond to your computer.
- Only contest2.cpp and imagePipeline.cpp have been changed from the original contest package.
