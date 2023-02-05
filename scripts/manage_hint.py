#! /usr/bin/env python

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
# from exprolab_ass3.srv import result, resultRequest, resultResponse
from exprolab_ass3.srv import Marker, MarkerRequest, MarkerResponse
from std_msgs.msg import Bool, Int32
from exprolab_ass3.msg import ErlOracle

hint_received=[]

def hint_callback(msg):

    global hint_received
    found = False

    if(msg.data > 10 and msg.data < 41):

        for i in hint_received:
            if i == msg.data:
                found = True
        if found == False:
            hint_received.append(msg.data)
            req_oracle = MarkerRequest()
            req_oracle.markerId = msg.data

            res_oracle = MarkerResponse()
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
