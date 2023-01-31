#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from exprolab_ass1.srv import hint, hintRequest, hintResponse
from exprolab_ass1.srv import correct_hyp, correct_hypRequest, correct_hypResponse


possible_hint = [ [1, 'who', 'Lorenzo'] , [1, 'what', 'can'] , [1, 'where', 'Kitchen'] ,
                  [2, 'who', 'Ciro'] , [2, 'what', 'knife'] ,
                  [3, 'who', 'Laura'] , [3, 'what', 'shotgun'] , [3, 'where', 'Livingroom'] , [3, 'where', 'Bedroom'] , 
                  [4, 'who', 'Zoe'] , [4, 'what', 'poison'] , [4, 'where', 'Bathroom'] ]


id_win = 1

hint_client=None
check_client=None

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
        # function called when exiting from the node, it can be blacking
        rand = random.randrange(11)
        hint_to_send = possible_hint[rand]
        req = hintRequest()
        req.ID = hint_to_send[0] 
        req.key = hint_to_send[1]
        req.value = hint_to_send[2]
        hint_client(req)
        print("I'm in look for hint")
        print(str(req.ID) + req.key + req.value)
        

        req_h = correct_hypRequest()
        res_h = correct_hypResponse()

        res_h = check_client(req_h)
        #print(res_h)
        
        if res_h.hypotesis == '':
            completed = False
        else:
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

        print("I'm in check result")
        correct = True

        if correct:
            return 'Goal'
        else:
            return 'Goto_new_room'
        
def main():
    rospy.init_node('FSM')

    global hint_client
    global check_client
    hint_client= rospy.ServiceProxy("/hint", hint)
    rospy.wait_for_service('/hint')

    check_client= rospy.ServiceProxy("/check", correct_hyp)
    rospy.wait_for_service('/check')
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
