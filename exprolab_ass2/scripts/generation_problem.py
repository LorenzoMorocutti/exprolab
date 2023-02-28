#! /usr/bin/env python

## @package exprolab_ass2
#
#  \file generation_problem.py
#  \brief this file implements the necessary calls to the rosplan server
#
#  \author Lorenzo Morocutti
#  \version 1.0
#  \date 12/02/2023
#  \details
#  
#  Subscribes to: <BR>
#	 
#
#  Publishes to: <BR>
#	 
#
#  Services: <BR>
#    
#  Action Services: <BR>
#
#  Client Services: <BR>
#  /rosplan_problem_interface/problem_generation_server
#  /rosplan_planner_interface/planning_server
#  /rosplan_parsing_interface/parse_plan
#  /rosplan_plan_dispatcher/dispatch_plan
#  /rosplan_knowledge_base/update
#    
#
#  Description: <BR>
#  This node implements the various calls to ROSplan to generate the plan run by the robot.
#  It requests feedback from the plan and produces new plans until
#  the goal is reached. It also updates the knowedge base.


import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import time
import actionlib
import random


def predicates_on_waypoint(wp):

##
#	\param wp: it is the waypoint I want to modify 
#	\return : None
# 	
#	This function use the UpdateService to add the predicate (notvisited) and
#   delete the predicate (checked) for the waypoint passed as argument

    #request to change the 'notvisited' predicate
    req=KnowledgeUpdateServiceRequest()
    req.update_type=0
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'notvisited'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', wp))	
    res=update(req)

    #request to change the 'checked' predicate
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'checked'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', wp))	
    res=update(req)



def update_knowledgeBase():

##
#	\param : None 
#	\return : None
# 	
#	This function use the UpdateService to delete the predicate (checked_complete)

    predicates_on_waypoint("w1")
    predicates_on_waypoint("w2")
    predicates_on_waypoint("w3")
    predicates_on_waypoint("w4")

    #request to change the 'checked_complete' predicate 
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'checked_complete'
    res=update(req)	



def rosplan_init():

##
#	\param : None
#	\return : None
# 	
#	This function initializes all the servers fro ROSplan

    global problem_generation_server, planning_server, parse_plan, dispatch_plan, update
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem_generation_server = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    planning_server = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parse_plan = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    dispatch_plan = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)	
    rospy.wait_for_service('/rosplan_knowledge_base/update')
    update= rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    print('all servers loaded')


def main():

    rospy.init_node('generation_problem')
    rosplan_init()
    success=False
    goal_reached=False

    # while cycle until we have a successful outcome
    while (success== False or goal_reached == False):
		# generate the problem
        response_pg=problem_generation_server()
        print('problem generated')
        time.sleep(1)

        # generate the plan
        response_pl=planning_server()
        print('plan generated')
        time.sleep(1)

        # parsing of the planner 
        response_pars=parse_plan()
        print('parse generated')
        time.sleep(1)

        # take feedback
        response_dis=dispatch_plan()
        print(response_dis.success)
        print(response_dis.goal_achieved)
        success= response_dis.success
        goal_reached= response_dis.goal_achieved
        
        # update the knowledge base
        update_knowledgeBase()
        time.sleep(1)

    print ('Plan terminated')

if __name__ == '__main__':
    main()