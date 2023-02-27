## Running the code
To run the code we actually just have to clone this branch of the repository in our catkin_ws, run the `catkin_make`, go to the scripts folder and digit:
```
chmod +x script1_name.py script2_name.py ...
```
for every .py file.

Then we just write in the terminal:
```
roslaunch exprolab_ass1 assignment.launch
```
to launch all the nodes.

## Working Hypotesis and Environment
The environment in which the robot moves is immaginary, so the robot's actions aren't real but just simulated. However, the node created are easily adaptable to a Gazebo simulation (for example) with the addition of some codes in the finite state machine.

### System's Features
The system is very modular, each node handles a different task: `oracle` gives the hints, `create_hypotesis` manages all the hint's aspects and `fsm` manage the robot's actions.

### System's Limitations 
The hints managed by the robot are complete (it can manage also malformed or empty hints, because I already know that it would be an aspect in the next assignments) but the ontology is not readable when updated and the terminal output is difficult to read if someone is not familiar with the smach output.

### Possible Techical Improvements
It would be nice to test this branch with a simulation environment instead of using simulated actions and it could be useful to have a more user friendly interface to manage the output of the various nodes. 