#!/usr/bin/env python

## @package exprolab_ass1
#
#  \file oracle.py
#  \brief Node that implements the behaviour of the oracle, it sends the hints and 
#  contains the winnin id 
#
#  \author Lorenzo Morocutti
#  \version 1.0
#  \date 12/2/2023
#  \details
#  
#  Subscribes to: <BR>
#       /newroom(boolean)
#
#  Publishes to: <BR>
#		/hint(ErlOracle.msg)
#
#  Services: <BR>
#       /result
#
#  Client Services: <BR>
#       /print_result
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#    This node implements the behaviour of a oracle: it gives the hints (chosen randomically
#    from the possible_hint array) and gives the solution (if the ID is correct, it confirms 
#    the game has ended, otherwise the game will continue).


import roslib
import rospy
import smach
import smach_ros
import time
import random
from exprolab_ass1.srv import hint, hintRequest, hintResponse
from exprolab_ass1.srv import correct_hyp, correct_hypRequest, correct_hypResponse
from exprolab_ass1.srv import result, resultRequest, resultResponse
from exprolab_ass1.srv import print_res, print_resResponse, print_resRequest
from std_msgs.msg import Bool
from exprolab_ass1.msg import ErlOracle

# global variables

possible_hint = [ [1, 'who', 'Lorenzo'] , [1, 'what', 'can'] , [1, 'where', 'Kitchen'] ,
                  [2, 'who', 'Ciro'] , [2, 'what', 'knife'] ,
                  [3, 'who', 'Laura'] , [3, 'what', 'shotgun'] , [3, 'where', 'Livingroom'] , [3, 'where', 'Bedroom'] , 
                  [4, 'who', 'Zoe'] , [4, 'what', 'poison'] , [4, 'where', 'Bathroom'],
                  [-1, "",""] , [-1, "",""] ,[-1, "",""]  ]


id_win = 1

hint_pub=None
print_client=None

def send_hint():

##
# \brief this function sends one random hint
# \param: None 
# \return: None
# This function choose a hint from the possible ones and and publishes it on the /hint topic

    global hint_pub
    rand = random.randrange(15)

    #load the field's hint to publish
    hint_to_send = possible_hint[rand]
    msg=ErlOracle()
    msg.ID = hint_to_send[0] 
    msg.key = hint_to_send[1]
    msg.value = hint_to_send[2]
    rospy.loginfo(msg)
    hint_pub.publish(msg) 




def newroom_callback(data):

##
# \brief this function activate send_hint()
# \param: data 
# \return: None
# This function activate the function send_hint() when is notificated we have arrived in a new room

    print("im in newroom_clbk")
    send_hint()




def result_clbk(req):

##
# \brief this function send the result to the state machine
# \param: req of the result type 
# \return: response with the winner's fields (if it is)
# This function requests the field of the complete hypotesis we have, check if it's the winning one 
# and, in thsi case, send the fields to the state machine to print the culprit 

    res = resultResponse()
    req_print = print_resRequest()
    res_print = print_resResponse()

    if req.ID == id_win:
        res.win = True
        req_print.ID = req.ID

        res_print = print_client(req_print)
        res.who = res_print.who
        res.what = res_print.what
        res.where = res_print.where
    
    return res


def main():
    global hint_pub, print_client

    rospy.init_node('oracle')
    #initialize the subscriber to the topic saying when it reaches the room
    rospy.Subscriber("/newroom", Bool, newroom_callback)

    hint_pub = rospy.Publisher('/hint', ErlOracle, queue_size=10)

    result_service=rospy.Service('/result', result , result_clbk)
    print_client=rospy.ServiceProxy('print_result', print_res)
    rospy.wait_for_service("/print_result")

    while not rospy.is_shutdown():
        rospy.sleep(15)
        rand = random.randrange(3)
        if rand==2:
            send_hint()

if __name__ == '__main__':
    main()