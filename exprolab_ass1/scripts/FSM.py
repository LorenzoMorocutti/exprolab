#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from exprolab_ass1.srv import hint, hintRequest, hintResponse
from exprolab_ass1.srv import correct_hyp, correct_hypRequest, correct_hypResponse
from exprolab_ass1.srv import result, resultRequest, resultResponse
from std_msgs.msg import Bool
from exprolab_ass1.msg import ErlOracle

id_win = 1

hint_client=None
check_client=None
room_pub=None

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


# define state Unlocked
class Goto_new_room(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['Look_for_hint'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
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
        room_pub.publish(True)
        #wait to see if any hint received

        rospy.sleep(2)
        
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
            hyp_received.append(temp[i])

        for i in hyp_received:
            if not (i in hyp_checked):
                completed = True            

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
        time.sleep(5)
        rospy.loginfo('Reached home! \n')

        return 'Check_result'


class Check_result(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['Goto_new_room','Goal'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking

        req=resultRequest()
        res=resultResponse()

        print("I'm in check result")

        for i in hyp_received:
            if not (i in hyp_checked):
                req.ID = i 
                hyp_checked.append(i)
                break

        res = result_client(req) 

        if res.win == True:
            correct = True
            rospy.loginfo("The killer was "+res.who+" in the "+res.where+" with the "+res.what)
        else:
            correct = False 

        if correct:
            return 'Goal'
        else:
            return 'Goto_new_room'
        
def main():
    rospy.init_node('FSM')

    global hint_client
    global check_client
    global room_pub, result_client
    hint_client= rospy.ServiceProxy("/hint", hint)
    rospy.wait_for_service('/hint')

    check_client= rospy.ServiceProxy("/check", correct_hyp)
    rospy.wait_for_service('/check')

    result_client= rospy.ServiceProxy("/result", result)
    rospy.wait_for_service('/result')
    
    room_pub = rospy.Publisher('/newroom', Bool, queue_size=10)

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
