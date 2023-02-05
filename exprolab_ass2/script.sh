#!/bin/bash
gnome-terminal --tab --title="gazebo" -- bash -c "roslaunch exprolab_ass2 simulation.launch"
sleep 1
gnome-terminal --tab --title="planner and ontology" -- bash -c "roslaunch exprolab_ass2 planner.launch"
sleep 1