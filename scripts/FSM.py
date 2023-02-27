#! /usr/bin/env python

## @package exprolab_ass3
#
#  \file FSM.py
#  \brief Node that implements the smach state machine for the third assignment
#
#  \author Lorenzo Morocutti
#  \version 1.0
#  \date 20/2/2023
#  \details
#  
#  Subscribes to: <BR>
#       /hint
#
#  Publishes to: <BR>
#		/cmd_vel
#
#  Services: <BR>
#       None
#
#  Client Services: <BR>
#       /hint
#       /check
#		/oracle_solution
#		/print_result
#
#  Action Services: <BR>
#       /move_base
#
#  Description: <BR>
#    This node implements the the finite state machine, with the smach library, that
#    manage the overall application. The logic is developed in four states and it's
#    very simple: I go to a new room (the robot has to check all the rooms and can't 
#    return to the same room twice) setting as a goal pose one of the rooms; then I 
#    look for a hint (read all the markers near the robot) moving the arm, in order 
#    to catch all the markers around the robot; if I have enough hint and a complete 
#    hypotesys, I go home; if the hypotesis is consistent and complete, I try to guess
#    the killer, where the murder has happened and with which weapon.

import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from exprolab_ass3.srv import hint, hintRequest, hintResponse
from exprolab_ass3.srv import correct_hyp, correct_hypRequest, correct_hypResponse
from exprolab_ass3.srv import print_res, print_resRequest, print_resResponse
from std_msgs.msg import Bool
from exprolab_ass3.msg import ErlOracle
from exprolab_ass3.srv import Oracle, OracleRequest, OracleResponse
import moveit_commander
import moveit_msgs.msg
import math

id_win = 1

hint_client=None
check_client=None
vel_pub = None

room_coordinates = [ [-4, -3], [-4, 2], [-4, 7], [5, -7], [5, -3], [5, 1] ]
  
checked_room = [0, 0, 0, 0, 0, 0]
hyp_checked=[]
hyp_received=[]


def hint_callback(msg):

##
# \brief function that receives a hint
# \param: msg of type EarloOracle.msg 
# \return: None
# This function make a hint request to receive a hint from the oracle

    req = hintRequest()
    req.ID = msg.ID
    req.key = msg.key
    req.value = msg.value
    hint_client(req)
    print("I received a hint")
    #print(str(req.ID) + req.key + req.value)'''


def arm_mov():

##
# \brief function that move the robotic arm
# \param: None 
# \return: None
# This function move the robotic arm thanks to the 'moveit' functions, it moves
# the arm to the position set in the 'goal' array

    goal = group_cmd.get_current_joint_values()
    goal[1] = 0
    goal[2] = 0
    goal[3] = 0
    group_cmd.go(goal, wait=False)
    time.sleep(10)
    goal[1] = math.pi/4
    goal[2] = math.pi/4
    goal[3] = -math.pi/2 + 0.05
    group_cmd.go(goal, wait=False)
    time.sleep(10)


def look_around():

##
# \brief function that manage the motion of the robotic arm
# \param: None 
# \return: None
# This function move the robotic arm low and high while rotating it,
# in order to catch all the markers with the camera

    velocity = Twist()
    velocity.angular.z = 0.75
    vel_pub.publish(velocity)
    arm_mov()
    rospy.sleep(10)
    velocity.angular.z = 0.0
    vel_pub.publish(velocity)
    



class Goto_new_room(smach.State):

##
# \brief this class implements the Go_to_new_room action
# \param: None 
# \return: the next state
# This class implements the action of going to a new room, setting as goal the next room 
# (chosen randomically)  and marking the rooms already visited

    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['Look_for_hint'])
        
    def execute(self, userdata):

        room_not_found = True
        room_to_do = False
        goal = MoveBaseGoal()

        # choose a random room that is not already visited
        for i in range(0, 6):
            if(checked_room[i] == 0):
                room_to_do = True
        
        if(room_to_do == False):
            for i in range(0, 6):
                checked_room[i] = 0

        while(room_not_found):
            index_room = random.randint(0, 5)
            if(checked_room[index_room] == 0):
                room_not_found = False
                checked_room[index_room] = 1

        #set the coordinates of the new room as goal       
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = room_coordinates[index_room][0]
        goal.target_pose.pose.position.y = room_coordinates[index_room][1]
        move_client.send_goal(goal)
        move_client.wait_for_result()

        rospy.loginfo('Reached a new room! \n')

        return 'Look_for_hint'




class Look_for_hint(smach.State):

##
# \brief this class implements the Looking_for_hint action
# \param: None 
# \return: the possible next state
# This class implements the action of looking for a hint, in brief it moves the robotic arm
# and check if it can make a complete and consistent hypotesis (in that case, it will go home)

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['Goto_new_room','Go_home'])
        
    def execute(self, userdata):
        global hyp_received, hyp_checked
        completed = False

        look_around()
        
        #see if given the new hint/hints there is a good hyp
        req_h = correct_hypRequest()
        res_h = correct_hypResponse()
    
        res_h = check_client(req_h)

        #split the fields of the hint at the '/' and remove ""
        temp = res_h.hypotesis.split("/")
        temp.remove("")
        print(temp)
        hyp_received.clear()

        #if it's not an hypotesis I already checked, save it
        for i in range(len(temp)):
            temp[i] = int(temp[i])

            if not (temp[i] in hyp_checked):
                hyp_received.append(temp[i])

        for i in hyp_received:
            if not (i in hyp_checked):
                completed = True
        rospy.loginfo("hypothesis to check"+ str(hyp_checked))            

        #if there is something to check, go home
        if completed:
            return 'Go_home'
        else:
            return 'Goto_new_room'
    

class Go_home(smach.State):

##
# \brief this class implements the Go_home action
# \param: None 
# \return: the next state
# This class implements the action of going home, setting as goal the home position 

    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['Check_result'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        goal = MoveBaseGoal()
        
        #set as goal, the coordinates of home
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 0.0
        goal.target_pose.pose.position.y = -1.0
        move_client.send_goal(goal)
        move_client.wait_for_result()
        rospy.loginfo('Reached home! \n')

        return 'Check_result'


class Check_result(smach.State):

##
# \brief this class implements the Check_result action
# \param: None 
# \return: exit if the hypotesis is correct, otherwise goto_new_room
# This class implements the action of checking the result, in brief it checks if the
# hypotesis is the winning one and, in that case, it prints the culprit, where the murder
# has happened and the weapond; otherwise, it goes to a new room

    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['Goto_new_room','Goal'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking

        #request to oracle to check the winning ID
        req=OracleRequest()
        res=OracleResponse()

        #print("I'm in check result")

        res = result_client(req) 
        rospy.loginfo("result to check: "+ str(res.ID))
        for i in hyp_received:
            if res.ID == i:
                req_print = print_resRequest()
                req_print.ID=i
                res_print = print_resResponse()
                res_print = print_client(req_print)
                rospy.loginfo("The killer was "+res_print.who+" in the "+res_print.where+" with the "+res_print.what)
                correct = True
            else:
                hyp_checked.append(i)
                correct = False 

        if correct:
            return 'Goal'
        else:
            return 'Goto_new_room'
        

        
def main():
    rospy.init_node('FSM')

    global vel_pub, result_client, move_client, print_client, check_client, hint_client, group_cmd, arm

    arm = moveit_commander.RobotCommander()
    name = "arm"
    group_cmd = moveit_commander.MoveGroupCommander(name)

    hint_client= rospy.ServiceProxy("/hint", hint)
    rospy.wait_for_service('/hint')

    check_client= rospy.ServiceProxy("/check", correct_hyp)
    rospy.wait_for_service('/check')

    result_client= rospy.ServiceProxy("/oracle_solution", Oracle)
    rospy.wait_for_service('/oracle_solution')

    print_client= rospy.ServiceProxy("/print_result", print_res)
    rospy.wait_for_service('/print_result')

    move_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    move_client.wait_for_server()
    
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber("/hint", ErlOracle, hint_callback)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Goal'])
    sm.userdata.sm_counter = 0



    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Goto_new_room', Goto_new_room(), 
                               transitions={'Look_for_hint':'Look_for_hint'})
        smach.StateMachine.add('Look_for_hint', Look_for_hint(), 
                               transitions={'Goto_new_room':'Goto_new_room', 
                                            'Go_home':'Go_home'})
        smach.StateMachine.add('Go_home', Go_home(), 
                               transitions={'Check_result':'Check_result'})
        smach.StateMachine.add('Check_result', Check_result(), 
                               transitions={'Goto_new_room':'Goto_new_room', 
                                            'Goal':'Goal'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('FSM', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
