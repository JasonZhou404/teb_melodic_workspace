# teb_melodic_workspace

**Dependencies:**

 * Melodic ROS
 * *navigation stack* and *teb_local_planner* package in melodic version
 * *stage* package in melodic version
 * a few python dependency like numpy, matplotlib

**Testing_procedures:**
1. run "catkin_make" in this folder
2. run "source /opt/ros/melodic/setup.bash" and "source devel/setup.bash" in every terminal used, better have it in your bashrc
3. run "roslaunch teb_local_planner_testing robot_carlike_in_stage.launch", and here we go (Clearly, Teb is doing crazy here)

![](results/0_0_1_setup.gif)

**TODO:**
1. Setup env with perfect localization and perception
2. Visualize the obstacle
3. Setup configuration accordng to paper experiment setups
4. Implement script to automatic run different configurations




