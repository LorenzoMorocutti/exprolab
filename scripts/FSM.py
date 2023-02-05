#!/usr/bin/env python

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
#import exp_moveit 
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
    req = hintRequest()
    req.ID = msg.ID
    req.key = msg.key
    req.value = msg.value
    hint_client(req)
    print("I received a hint")
    #print(str(req.ID) + req.key + req.value)'''

def arm_mov():
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
    velocity = Twist()
    velocity.angular.z = 0.75
    vel_pub.publish(velocity)
    arm_mov()
    rospy.sleep(10)
    # velocity.angular.z = -0.75
    # vel_pub.publish(velocity)
    # rospy.sleep(30)
    velocity.angular.z = 0.0
    vel_pub.publish(velocity)
    


# define state Unlocked
class Goto_new_room(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['Look_for_hint'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        room_not_found = True
        room_to_do = False
        goal = MoveBaseGoal()

        for i in range(0, 6):
            if(checked_room[i] == 0):
                room_to_do = True
        
        if(room_to_do == False):
            for i in range(0, 6):
                checked_room[i] = 0

        while(room_not_found):
            index_room = random.randint(0, 5)
            if(checked_room[index_room] ==0 ):
                room_not_found = False
                checked_room[index_room] = 1
        
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = room_coordinates[index_room][0]
        goal.target_pose.pose.position.y = room_coordinates[index_room][1]
        move_client.send_goal(goal)
        move_client.wait_for_result()

        rospy.loginfo('Reached a new room! \n')

        return 'Look_for_hint'



# define state Unlocked
class Look_for_hint(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['Goto_new_room','Go_home'])
        
    def execute(self, userdata):
        global hyp_received, hyp_checked
        completed = False
        # function called when exiting from the node, it can be blacking
        #wait to see if any hint received

        look_around()
        
        #see if given the new hint/hints there is a good hyp
        req_h = correct_hypRequest()
        res_h = correct_hypResponse()
    
        res_h = check_client(req_h)
        temp = res_h.hypotesis.split("/")
        temp.remove("")
        print(temp)
        hyp_received.clear()

        for i in range(len(temp)):
            temp[i] = int(temp[i])

            if not (temp[i] in hyp_checked):
                hyp_received.append(temp[i])

        for i in hyp_received:
            if not (i in hyp_checked):
                completed = True
        rospy.loginfo("hypothesis to check"+ str(hyp_checked))            

        if completed:
            return 'Go_home'
        else:
            return 'Goto_new_room'
    

class Go_home(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['Check_result'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        goal = MoveBaseGoal()
        
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 0.0
        goal.target_pose.pose.position.y = -1.0
        move_client.send_goal(goal)
        move_client.wait_for_result()
        rospy.loginfo('Reached home! \n')

        return 'Check_result'


class Check_result(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['Goto_new_room','Goal'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking

        req=OracleRequest()
        res=OracleResponse()

        print("I'm in check result")

        res = result_client(req) 
        rospy.loginfo("result to check: "+ str(res.ID))
        for i in hyp_received:
            if res.ID == i:
                req_print = print_resRequest()
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
