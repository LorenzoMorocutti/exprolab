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

        req.args= ['/root/ros_ws/src/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
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
            if weapons[j]==data[2]:
                received=True
        # if it has never been received I add it to the list
        if received==False:
            weapons.append(data[2])

    # check 'where'
    if data[1]=='where':
        # checks every element of the array locations
        for i in range(len(locations)):
            if locations[k]==data[2]:
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
#    First controls the class of the entity, then it adds it to the ontology thanks to another
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


##
#	\brief This function implements the /results server
#	\param : req: it is an integer that contains the ID of an hypothesis
#	\return : resp: it is composed of three strings, one for each  field of the hypothesis	
#   This server retrieves the field of a requested hypothesis,
#   identified by the ID that is sent as a request. 
#   The fields are retrieved in the armor server and sent back on the response.
	
def print_clbk(req):
	# I initialize the response variable
    resp=print_resResponse()
    # save the value retrieved in the field what in a temporary variable
    what=str(look_hypothesis(str(req.ID), 'what')[0])
    # save the value retrieved in the field who in a temporary variable
    who=str(look_hypothesis(str(req.ID), 'who')[0])
    # put the value for who in the response message
    resp.who=who
    # save the value retrieved in the field where in a temporary variable
    where=str(look_hypothesis(str(req.ID), 'where')[0])
    # put the value for what in the response message
    resp.what=what
    # put the value for where in the response message
    resp.where=where
    return resp
	

	
	
##
#	\brief It checks if there is at least one hypothesis complete and not inconsistent
#	\param None
#	\return : returns 1 if the hypothesis is complete and not inconsistent
#    returns 0 if it is either incomplete or inconsistent.
# 	
#	This function calls the armor server twice, the first time it retrieves all
#   the complete hypothesis and it checks if there is at least one hypothesis that is
#   complete and not inconsistent
def check_complete_consistent(req):
    try:
        completed=[]
        inconsistent=[]
        temp=""
        # set the request for the armor server check all the completed hypothesis
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['COMPLETED']
        # send the request
        msg = armor_service(req)
        # save the response of the server
        res=msg.armor_response.queried_objects
        # clean the results by removing usless parts
        res_final=clean_queries(res)
        completed=res_final
        # set the request for the armor server check all the inconsistent hypothesis
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['INCONSISTENT']
        # send the request
        msg = armor_service(req)
        # save the response of the server
        res=msg.armor_response.queried_objects
        # clean the results by removing usless parts
        res_final=clean_queries(res)
        inconsistent=res_final
        # if the hypothesis is completed AND inconsistent return 1
        # for every element in the array completed
       
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
	
	
 

##
#	\brief this function updates the ontology
#	\param : None
#	\return : None
# 	
#	 This function calls the armor server in order to run the reasoner
#    and make implicit knowledge explicit       
def reason():
    try:
        # set the request for the armor server to use the reasoner
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REASON'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= []
        # send the request
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)		
 
##
#	\brief This function specifies that all elements of a class are different
#	\param class : of type string it is the class of which element I want to make disjoint
#	\return : None
# 	
#	This function calls the armor server and by sending specific commands it
#   specifies that all entities inside the class passed as parameter are 
#   disjoint and different   
def disjoint(id_class):
    try:
        # set the request for the armor server to use the reasoner
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [id_class]
        # send the request
        msg = armor_service(req)		 
    except rospy.ServiceException as e:
        print(e)        

##
#	\brief This function cleans the query returned from the ontology
#	\param query: the list of strings that needs to be cleaned
#	\return : query, the cleaned query
# 	
#	This function, for each element of the list passed as input it splits
#   the strings at the char '#' and takes only what is after. Then it removes
#   the last element of the remaing string. The received query is of type 
#   <http://www.emarolab.it/cluedo-ontology#rope> and in this example
#   we want to extract rope
def clean_queries(query):
    # for every element of the list received as input 
    for i in range(len(query)):
        temp=query[i]
        # split at the character '#'
        temp=temp.split('#')
        # save the lenght of the list returned after the split
        index=len(temp)
        # take only the last element ( lenght -1 ) 
        temp=temp[index-1]
        # saves it in the query, overwriting the one received and 
        # eliminating the last character
        query[i]=temp[:-1]
    return query

         

##
#	\brief This function adds an hypothesis to the ontology
#	\param ID : of type string it is the ID of the hypothesis I want to add to
#	\param class_type : string representing the type of information I want to add
#   \param name : of type string it is the name of the information I want to add
#	\return : None
# 	
#	By calling the armor server with the proper commands I add an entity to
#   a given hypothesis. 
def add_hypothesis(ID,class_type,name):
    try:
        # set the request for the armor server to add an entity to an hypothesis
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID,name]
        # send the request
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)	

##
#	\brief It retrieves from an hypothesis a field
#	\param ID : of string type it is the hypothesis I want to check 
#   \param class_type : the property of the hypothesis that I want to retrieve
#	\return : res_final a string with the name of the entity I retrieved
# 	
#	This funciton calls the armor server to see in a given hypothesis identified
#   by its ID one field, identified by the class_type.
def look_hypothesis(ID,class_type):
    try:
        # set the request for the armor server to check one field (identified by class_type)
        # of an hypothesis (identified by an ID)
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID]
        # send the request
        msg = armor_service(req)
        # save the response of the server
        res=msg.armor_response.queried_objects
        #print(res)
        # clean the results by removing usless parts
        res_final=clean_queries(res)
        return res_final
    except rospy.ServiceException as e:
        print(e)   

##
#	\brief It checks if the hint received is already saved in the hypothesis
#	\param ID: of type string, the ID of the hypothesis I want to check 
#	\param class_type: of type string, the type of instance I want to check
#       if already present
#	\param name : of type string, the name of the entity I want to check
#	\return : None
# 	
#	This function checks if a hint ( composed of an ID, class_type and name) 
#   is already present in an hypothesis. If it is not present it adds it 
#   to the ontology.
def check_owl(ID,class_type,name):
    try:
        # it retrieves from the hypothesis ID the field identified by class_type
        res_final=look_hypothesis(ID, class_type)
        find = False
        # if the name retrieved is different from the name received I add the hypothesis
        # it adds it even if the retrieved field is empty
        for i in res_final:

            if i == name:
                find = True

        if find == False:
            add_hypothesis(ID,class_type,name)
            reason()
    except rospy.ServiceException as e:
        print(e)      



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