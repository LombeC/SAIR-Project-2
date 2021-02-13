# Project 2 - Task 1
### Lombe


### **Compiling and running du_chileshe_lombe_p2_t1**

1. Prior to starting, make sure ```roscore```
is running.

2. Place the project in your catkin workspace
```bash
cd ~/catkin_ws/src/du_chileshe_lombe_p2_t1/
```
Alternatively, you can pull it from github
```bash
git clone https://github.com/LombeC/SAIR-Project-2.git
```
3. Navigate to the catkin folder and build the project
```bash
cd ~/catkin_ws 	# Navigate to the catkin_ws
catkin_make 	# Build
```
4. Make sure the build executable is on your path. Execute the following command while inside *catkin_ws* :

```bash
source ./devel/setup.bash
```

5. Then run the following in separate terminals 
```bash
roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
```
After gazebo launches, be sure to set the starting pose of the robot to (0,0,0)
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
roslaunch du_chileshe_lombe_p2_t1 turtlebot3_navigation.launch
rosrun du_chileshe_lombe_p2_t1 my_transform_listener 
```
You should expect to see the robot move from the starting point to a goal location, then to a second goal location.
The goal locations are hardcoded in the source code. 

Also, you should expect to the current location of the robot logged onto the terminal