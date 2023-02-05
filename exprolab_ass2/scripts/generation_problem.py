#! /usr/bin/env python

import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import time
import actionlib
#import exprob_assignment2.msg
import random

# mettere i not visited

def update_waypoint(waypoint):
    req=KnowledgeUpdateServiceRequest()
    req.update_type=0
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'notvisited'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', waypoint))	
    result=update(req)


    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'checked'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', waypoint))	
    result=update(req)

# cancellare complete

def update_knowledgeBase():
    update_waypoint("w1")
    update_waypoint("w2")
    update_waypoint("w3")
    update_waypoint("w4")
    
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'checked_complete'
    result=update(req)	

##
#	\brief This function initializes all the server used for the planning part
#	\param : 
#	\return : None
# 	
#	This function initializes all the servers and waits for all of them to be 
#   running before moving forward
def initialization():
    global problem_generation, planning, parsing, dispatch, update
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server',Empty)
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    planning = rospy.ServiceProxy('/rosplan_planner_interface/planning_server',Empty)
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parsing = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    dispatch = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan',DispatchService)	
    rospy.wait_for_service('/rosplan_knowledge_base/update')
    update= rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    #print(rospy.get_param('random_hint'))
    print('all servers loaded')

def main():
    global pub_, active_, act_s
    rospy.init_node('generation_problem')
    initialization()
    success=False
    goal=False


    # until the feedback from the planner is not true
    while ( success== False or goal == False):
		# generate the problem
        response_pg=problem_generation()
        print('problem generates')
        time.sleep(1)
        # generate the plan
        response_pl=planning()
        print('plan generates')
        time.sleep(1)
        # parse the plan 
        response_pars=parsing()
        print('parse generates')
        time.sleep(1)
        # read the feedback
        response_dis=dispatch()
        print(response_dis.success)
        print(response_dis.goal_achieved)
        success= response_dis.success
        goal= response_dis.goal_achieved
        # update the knowledge base
        update_knowledgeBase()
        time.sleep(1)
    print ( 'SUCCESS')

if __name__ == '__main__':
    main()