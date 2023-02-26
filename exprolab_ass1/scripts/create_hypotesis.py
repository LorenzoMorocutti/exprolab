#! /usr/bin/env python

## @package exprolab_ass1
#
#  \file create_hypotesis.py
#  \brief This node does all the work involving the hints received
#
#  \author Lorenzo Morocutti
#  \version 1.0
#  \date 12/2/2023
#  \details
#  
#  Subscribes to: <BR>
#       None
#
#  Publishes to: <BR>
#		None
#
#  Services: <BR>
#       /hint
#		/check
#		/print_result
#
#  Client Services: <BR>
#       /armor_interface_srv
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#    This node implements the node that receives the hint from the oracle and 
#    saves it in the ontology. The armor node will tell us if the hint is malformed or valid. 
#    It also implements the checking of the completeness and consistency of the hypothesis.
#    Eventually retrieves the field of an hypothesis from the ontology, given an ID.

import copy
import math
import sys
import time
import geometry_msgs.msg
import numpy as np
import rospy
from std_msgs.msg import String, Int32
from armor_msgs.msg import * 
from armor_msgs.srv import * 

from exprolab_ass1.srv import hint, hintResponse
from exprolab_ass1.srv import correct_hyp,correct_hypResponse
from exprolab_ass1.srv import print_res, print_resRequest, print_resResponse


#global variables
people=[]
weapons=[]
locations=[]
armor_service = None
pub= None
complcons=[]
retrievedId=[]
oracle_service = None



def loading_owl():

##
#	\brief This function just loads our ontology
#	\param : None 
#	\return : None	
#	This function loads the cluedo ontology from the website of the emarolab
#   with the right armor command. We need to insert the right path to the ontolgy
#   (in my case: /root/ros_ws/src).

    try:
        # request to armor
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'LOAD'
        req.primary_command_spec= 'FILE'
        req.secondary_command_spec= ''

        req.args= ['/root/ros_ws/src/exprolab_ass1/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        # send the message on the server
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)





def hint_clbk(req):

##
#	\brief This function implements the server service for /hint
#	\param : req, the hint received from oracle
#	\return : hintResponse, a boolean true if the hint isn't malformed or already received 	
#	This function does all the checkings on the hint: if the hint received is correct
#   or there are some missing/malformed fields. If the hint is correct we also
#   check if it has already been received, in the ontology, and, if not, saves it

    #print('message received')
    res = hintResponse()
    hint_received=[]

    # check on ID
    if str(req.ID)=="" or req.ID==-1:
        print('id is empty or malformed')
        return hintResponse(False)

	# check on key
    if str(req.key)=="" or str(req.key)=='0' or str(req.key)=='-1' or (str(req.key)!='who' and str(req.key)!='what' and str(req.key)!='where'):
        print('key is empty or malformed')
        return hintResponse(False)

	# check on value
    if str(req.value)=="" or str(req.value)=='0' or str(req.value)=='-1':
        print('value is empty or malformed')
        return hintResponse(False)

	# I save the hint on a array as string (otherwise it could be difficult to send it in a unique message)
    hint_received.append(str(req.ID))
    hint_received.append(str(req.key))
    hint_received.append(str(req.value))

    print(hint_received[0])
    print(hint_received[1])
    print(hint_received[2])

    # check if the hint has been already received
    rec=already_received(hint_received)

    # if it was never received before
    if rec==False:
        # update ontology
        add_owl(hint_received[2],hint_received[1])

    # check if the hint's istance is already in the ontology	    
    check_owl(hint_received[0], hint_received[1], hint_received[2])

    return hintResponse(True)



  
def already_received(data):

##
#	\brief This function checks if the hint has already been received
#	\param data: a string with all the hint's fields
#	\return : received, a boolean, false if the hnt has never been received	
#	 In this function, we check if the data, consisting of a string, has ever been received. 
#    If already present, we will skip it, otherwise we have to save it. 

    global people, weapons, locations
    received=False
    i=0
    # check 'who'
    if data[1]=='who':
        # checks every element of the array 'people'
        for i in range(len(people)):
            if people[i]==data[2]:
                received=True
        # if it has never been received I add it to the list
        if received==False:
            people.append(data[2])

    # check 'what'
    if data[1]=='what':
        # checks every element of the array 'weapons'
        for i in range(len(weapons)):
            if weapons[i]==data[2]:
                received=True
        # if it has never been received I add it to the list
        if received==False:
            weapons.append(data[2])

    # check 'where'
    if data[1]=='where':
        # checks every element of the array locations
        for i in range(len(locations)):
            if locations[i]==data[2]:
                received=True
        # if it has never been received I add it to the list
        if received==False:
            locations.append(data[2])

    return received   



     
def add_owl(name, class_type):

##
#	\brief This function adds an hint to the ontology
#	\param name : string representing the name of the instance
#   \param class_type: string representing the type of the class
#	\return : None
# 	
#	 This function adds one hint to the ontology, translating it in the owl formalism.
#    First controls the class of the entity, then it adds it to the ontology thanks to an
#    armor command. In the end it reasons, to create all the dependencies of the ontology
#    with the disjuncted property (we have to explicitly say two entities are separate). 

    try:
        # from the class_type (who,what,where) find the entity_class (PERSON,WEAPON,PLACE)
        id_class=correspondent_type(class_type)

        # request to add an instance of a class to armor  
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [name, id_class]

        msg = armor_service(req)
        res=msg.armor_response

        # reasoner of the ontlogy
        reason()

        # set disjoint bond
        disjoint(id_class)

        # reason again after disjoint
        reason()

    except rospy.ServiceException as e:
        print(e)




def correspondent_type(class_type):

##
#	\brief this function returns the class_name corresponding to the class_type
#	\param class_type : a string, can be who, what or where 
#	\return : The class name
# 	
#	This function checks the type of the class given as input parameter and
#   returns the string with the corresponding class name.

    if class_type=='who':
        return 'PERSON'
    if class_type== 'what':
        return 'WEAPON'
    if class_type=='where':
        return 'LOCATION' 





def check_owl(ID,class_type,name):

##
#	\brief It checks if the istance is already in the ontology
#	\param ID: string, ID of the hypothesis I want to check 
#	\param class_type: string, the type of instance I want to check
#       if already present
#	\param name : string, the name of the entity to check
#	\return : None
# 	
#	This function checks if a hint is already present in the ontology. 
#   If it is not there it's been added.

    try:
        # it retrieves from the ID the class_type
        retrieve=look_for_hyp(ID, class_type)
        find = False
        # if the name retrieved isn't there I add the hypothesis 
        for i in retrieve:

            if i == name:
                find = True

        if find == False:
            add_hyp(ID,class_type,name)
            reason()
    except rospy.ServiceException as e:
        print(e)   





def add_hyp(ID,class_type,name):

##
#	\brief This function adds an hyp to the ontology
#	\param ID : string, the ID of the hypothesis to be added
#	\param class_type : string, represents the type of information
#   \param name : string, it's the information
#	\return : None
# 	
#   This function expoits, again, the Armor server to add an hyp (we give 
#   all the information about it, for the args)

    try:
        # request to the armor server
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID,name]
        # send request
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)





def look_for_hyp(ID,class_type):

##
#	\brief It retrieves from a ID a field
#	\param ID : string; it represents the hypotesis I want to check 
#   \param class_type : the characteristic I want to retrieve
#	\return : retrieve, string, intuitively the name of the entity I retrieved
# 	
#	This funciton calls the armor server to see the hypothesis relative
#   to a field, identified by the ID.

    try:
        # request to armor server to check the field class_type of an hypothesis
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID]

        # send the request
        msg = armor_service(req)

        # save response
        res=msg.armor_response.queried_objects

        # remove the useless parts from the retrieving
        retrieve=remove_queries(res)
        return retrieve
    except rospy.ServiceException as e:
        print(e)      




def remove_queries(owl_res):

##
#	\brief This function removes the query from the retrieves from the ontology
#	\param owl_res: the response from the Armor server
#	\return : owl_res, the cleaned response
# 	
#	This function splits the string, received as input, at the character '#' 
#   and saves only the part after that. The received string is something like 
#   <http://www.emarolab.it/cluedo-ontology#hammer> (we want to extract 'hammer', 
#   without the '>')

    for i in range(len(owl_res)):
        # we use an intermediate temporary variable
        temp=owl_res[i]
        # split at '#'
        temp=temp.split('#')
        # save the lenght of the array (it's equal to 2)
        index=len(temp)
        # take only the second element (hammer>) 
        temp=temp[index-1]
        # overwriting the one received, eliminating '>'
        owl_res[i]=temp[:-1]
        
    return owl_res




      

def reason():

##
#	\brief this function calls the reasoner for the ontology
#	\param : None
#	\return : None
# 	
#	 This function calls the armor server to run the reasoner, which
#    task is to explicit all the relations between the istances in the ontology 

    try:
        # request to armor server to run the reasoner
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REASON'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= []
        # send request
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)




 
def disjoint(id_class):

##
#	\brief This function calls the disjoint function
#	\param id_class : string, the class of the elements I want to do the disjoint
#	\return : None
# 	
#	This function calls the armor server to do a disjoint of all the elements 
#   belonging to the class passed as argument

    try:
        # request to the armor server to use the disjoint
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [id_class]
        # send request
        msg = armor_service(req)		 
    except rospy.ServiceException as e:
        print(e) 






def check_complete_consistent(req):

##
#	\brief Callback function to check if there is at least one hyp complete and consistent
#	\param req, a boolean
#	\return : returns a string of the complete and consistent hyps separate by a '/'
# 	
#	This function calls the armor server two times: first time to retrieve the
#   complete hypothesis; second to check if there is at least one hyp 
#   complete and consistent

    try:
        # init of the arrays
        completed=[]
        inconsistent=[]
        temp=""
        # first request to the armor server
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['COMPLETED']
        # send request
        msg = armor_service(req)
        # save response
        res=msg.armor_response.queried_objects
        
        # clean the responses by removing usless parts
        retrieve=remove_queries(res)
        completed=retrieve

        # second request to the armor server
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['INCONSISTENT']
        # send request
        msg = armor_service(req)
        # save response 
        res=msg.armor_response.queried_objects
        
        # clean the results by removing usless parts
        retrieve=remove_queries(res)
        inconsistent=retrieve
        
        # if the hypothesis is completed and not inconsistent saves
        # the id of the hyp in the array completed
        for i in completed:
            find = False
            for j in inconsistent:
                if i == j:
                    find = True
            if find == False:
                temp = temp + "/" + i
            
        return temp

    except rospy.ServiceException as e:
        print(e)





	
def print_clbk(req):

##
#	\brief Callback function to send the possible final result to print
#	\param : req: integer that represents the ID the hyp
#	\return : res: it is composed of three strings, one for each  field of the hypothesis	
#   This fucntion retrieves the fields of a requested hypothesis, supposedly the winning one 

	# init of the response variable
    res=print_resResponse()
    # retrieve the value of the field 'what'
    what=str(look_for_hyp(str(req.ID), 'what')[0])
    # put the value for 'what' in the response
    res.what=what

    # retrieve the value of the field 'who'
    who=str(look_for_hyp(str(req.ID), 'who')[0])
    # put the value for 'who' in the response
    res.who=who

    # retrieve the value of the field 'where'
    where=str(look_for_hyp(str(req.ID), 'where')[0])
    # put the value for 'where' in the response
    res.where=where
    return res
	


def main():
  global  armor_service, pub, oracle_service

  rospy.init_node('node_hint')
 
  armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
  rospy.wait_for_service('armor_interface_srv')

  service=rospy.Service('/hint', hint, hint_clbk)
  service=rospy.Service('/check', correct_hyp, check_complete_consistent)
  res_service=rospy.Service('/print_result', print_res, print_clbk)

  loading_owl()
  rospy.spin() 

  
if __name__ == '__main__':
  main()        