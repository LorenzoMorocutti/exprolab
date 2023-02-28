## Running the code
To run the code we have to follow some passages, after installing the `Moveit` library in the src folder of the catkin space:
* first: clone the respository
```
git clone https://github.com/LorenzoMorocutti/exprolab.git
cd exprolab
git checkout assignment2
```
* second: we move the two folders `exprolab_ass2` and `exp_moveit` in the src of the catkin workspace;
* third: go to the `scripts` folder and make the .py files executable
```
chmod +x script1_name.py script2_name.py ...
```
* fourth: go to the src folder and digit `catkin_make`


Then we just digit in the terminal, in two different tabs:
```
roslaunch exprolab_ass2 simulation.launch
roslaunch exprolab_ass2 planner.launch
```
to launch all the nodes.

Alternatively, if you have installed `gnome` on your system, you can just go the exprolab_ass2 folder and digit:
```
source script.sh
```

The `exp_moveit` folder is the folder containing all the scipts and cpp files needed to maneuver the robot od the simulation.

## Working Hypotesis and Environment
The environment in which the robot moves is a Gazebo simulation. In this case the hints are given by the author professor [Carmine Recchiutto](http://github.com/CarmineD8), so I just had to deal with them (by receiving them on /oracle_hint). ALl about the hint remains valid from the first assignment.

### System's Features
The architecture is definitely modular (let's just think about all the pddl actions that execute a single task, so we can change a small piece of the application modifying one of them). The system is very robust thanks to the ROSplan features that allow the software to replan in the case something unexpected (or just if it doesn't reach the goal the first time) happens.

### System's Limitations 
The hints managed by the robot aren't modifiable (they are, but the goal of the assignment is to manage hints not written by the user) but the ontology is not readable when updated and the terminal output is difficult to read if someone is not familiar with the outputs (using gnome, it's easier to read the output of the simulation and the planner). The simulation is very slow so it's very difficult to debug the code (there are definitely some shortcuts, for example, from the terminal we can publish hints on the topic /oracle_hint to give the robot a complete hypotesis directly to test the check_complete and check_result).
I want to emphasize also a curious aspect: I tried to run the code on a different device (a pc in the laboratory) because more powerful but I encountered some problems: after the first execution of the movement of the arm with moveit, the process relative to simulation.cpp crashed every time, so I wasn't able to test the code on another device (it's curious because in the third assignment we also use moveit but, in this case, the simulation is clean). 

### Possible Techical Improvements
An ovious improvement is to test the code with other devices to confirm what is the problem. Other improvements concern the velocity of the simulation that could be way faster if the go_to_point and moveit codes were optimized (in particular moveit is very slow).