# Tutorial on simple grasping pipeline

This tutorial focuses on navigating around ROS, MoveIt (the motion planner) and GraspIt (the grasp generator) to generate high-quality grasps from the PR2, and visualize them in Gazebo. 

## Obtaining Packages
You will need an installation of ROS. This tutorial is tested on jade, but it should also work for previous distros like indigo. Most of the CLIC machines at Columbia have ROS installed. Create a folder called *GraspDemo*, and a subdirectory called *src*. You will need the following packages, which can be obtained using the following links:

* **pr2_simulator**
```bash
$ git clone https://github.com/PR2/pr2_simulator.git 
```
* **GraspIt, graspit_commander, graspit_interface**
```bash
$ git clone https://github.com/graspit-simulator/graspit-ros --recursive
$ git clone git@github.com:CURG/graspit_commander.git
$ git clone git@github.com:CURG/graspit_interface.git
```
* **MoveIt and MoveIt_commander** (most CLIC machines already have MoveIt installed)
```bash
$ sudo apt-get install ros-indigo-moveit-ros
$ sudo apt-get install ros-indigo-pr2-moveit-config
```

You can place these packages in the src folder.

Note: ```sudo apt-get install``` places a compiled version of the package in ```/opt/ros/<version>```, whereas git clone places the source code in your current directory, which you could then modify and build manually. 

## Installation
If you already have some or all of these packages, just source those directories prior to running ```catkin_init_workspace```. Note that ```catkin_init_workspace``` is run when creating the workspace for the first time, not when re-compiling code. As you add new packages, just source those directories using ```source <path to package>``` and then run ```catkin_make```. Note that catkin is a thin wrapper around cmake. 

Before you run ```bash catkin_init_workspace```, you must source ROS by running: 
```bash
$ source /opt/ros/indigo/setup.bash
```
Compile the code using:
```bash
$ catkin_make
```
This tutorial focuses on grasping and execution, but does not cover vision. So there needs to be a way to transfer the model from Gazebo to the GraspIt planning scene. For now, you can add the model to be grasped to the graspit planning scene. Do the following:

1. place *cup.xml* in *src/graspit-ros/graspit/graspit_source/models/objects*

2. place *cup.ply* in *src/graspit-ros/graspit/graspit_source/models/objects*

3. place *pr2_mug.launch* in *src/graspit-ros/graspit/graspit_source/worlds*

4. place *plane.xml* in *src/graspit-ros/graspit/graspit_source/models/obstacles*

5. place *plane.ply* in *src/graspit-ros/graspit/graspit_source/models/obstacles*

#### ROS-side stuff
In *src*, add the following two folders from the *All_files.zip*: *scripts* and *system_launch*. 

This tutorial includes a python library of functions to execute grasps on an object, called *utils_grasp.py*, as well as a python interface to this library called *grasp_execution.py*. Both of these files are in *src/scripts*. 

*utils_grasp.py* includes functions to solve many common problems such as moving the arms, closing the gripper, and moving the head & torso. You would need to modify the ```spawn_mug()``` function, in particular the path to the model to be spawned. 

The Python interface is simple but powerful. Currently, the user cycles through the 20 best grasps from GraspIt, and selects which one to execute by visual evaluation. A function to automatically assess reachability is also included. Successful grasps return the object back to the original location, so you can try another grasp. If the cup moves because of a bad grasp, you can grab the cup in gazebo, throw it out of the way, and spawn a new one. Note that this step is necessary because this tutorial does not include vision so the initial coordinates of the cup are hardcoded. Here is the command to spawn a new model, which you can run by opening a new terminal:
```bash
$ rosrun gazebo_ros spawn_model -file <path_to_cup>/cup.urdf -urdf -model New_Model_Name -x .6 -y -.1 -z 0.55 -Y -1.57
```
Note that *New_Model_Name* must be unique for each model you spawn. Replace it with any string.



## Planning and executing grasps
To launch Gazebo with the PR2 inside, as well as GraspIt and MoveIt, execute: 
```bash
$ source devel/setup.bash
roslaunch system_launch pr2_mug.launch
```

Then open a new terminal and run: 
```bash
$ source devel/setup.bash
$ cd src/scripts
$ python grasp_execution.py
```
You can keep opening new terminals and send commands to ROS from any terminal. Just be sure to run ```source devel/setup.bash``` for each terminal. 



## Additional help:
[Humanoid Robotics Course](https://github.com/HumanoidRobotics/pr2_example_ws)

## Useful commands
* ```$ history | grep <partial name of command>``` to help retrace your steps
* ```$ ipython```  -> start an IPython session from the command line
* ```import IPython```
```IPython.embed()``` -> Within a Python script, stops execution and helps you check variables, test subsequent lines of code without crashing the program. Very useful for debugging python code. 
* ```env | grep -i ros``` -> Check the ROS package path 

## Written by
Vaibhav Vavilala SEAS '17

12/2016

## Thanks
Jake Varley 

Iretiayo Akinola

Peter Allen