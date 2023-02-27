#!/usr/bin/env python

## @package exprolab_ass3
#
#  \file manage_hint.py
#  \brief Node that implements the manager og the hint associated to the markers
#
#  \author Lorenzo Morocutti
#  \version 1.0
#  \date 20/2/2023
#  \details
#  
#  Subscribes to: <BR>
#       /marker_publisher/detected_id
#
#  Publishes to: <BR>
#		None
#
#  Services: <BR>
#       None
#
#  Client Services: <BR>
#       /hint
#       /oracle_hint
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#    This node implements the behaviour of tha manager of the markers present in the simulated
#    world. It retrieves the hint associated a marker.

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
from exprolab_ass3.srv import Marker, MarkerRequest, MarkerResponse
from std_msgs.msg import Bool, Int32
from exprolab_ass3.msg import ErlOracle

hint_received=[]

def hint_callback(msg):

##
# \brief function that retrieve a hint
# \param: msg of type Int32
# \return: None
# This function retrieve the hint associated to the marker_id

    global hint_received
    found = False

    #if the id of the marker is higher than 10 and lower than 41
    if(msg.data > 10 and msg.data < 41):

        #if the hint has been already received, don't take it again
        for i in hint_received:
            if i == msg.data:
                found = True
        
        #if the hint hasn't been already received, save it
        if found == False:
            hint_received.append(msg.data)
            req_oracle = MarkerRequest()
            res_oracle = MarkerResponse()

            req_oracle.markerId = msg.data
            res_oracle = oracle_client(req_oracle)

            rospy.loginfo(res_oracle)

            req = hintRequest()
            req.ID = res_oracle.oracle_hint.ID
            req.key = res_oracle.oracle_hint.key
            req.value = res_oracle.oracle_hint.value
            hint_client(req)
            print("I received a hint")




def main():
    rospy.init_node('manage_hint')

    global oracle_client, hint_client

    hint_client= rospy.ServiceProxy("/hint", hint)
    rospy.wait_for_service('/hint')

    rospy.Subscriber("/marker_publisher/detected_id", Int32, hint_callback)
    oracle_client = rospy.ServiceProxy('oracle_hint', Marker)
    rospy.wait_for_service('/oracle_hint')

    # Wait for ctrl-c to stop the application
    rospy.spin()


if __name__ == '__main__':
    main()
