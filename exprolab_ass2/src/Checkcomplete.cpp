/** @ package exprolab_ass2
* 
*  \file Checkcomplete.cpp
*  \brief implements the 'check_complete_hypotesis' action
*
*  \author Lorenzo Morocutti
*  \version 1.0
*  \date 12/02/2023
*  \details
*   
*  Subscribes to: <BR> 
*    None
*
*  Publishes to: <BR>
*	 None
*
*  Services: <BR>
*    None
* 
*   Client Services: <BR>
*    /check
*    
*
*  Action Services: <BR>
*    None
*
*  Description: <BR>
*  This node implements the 'check_complete_hypotesis' action to check if the passed hypotesis is complete and consistent. 
*/

#include "exprolab_ass2/ActionInterface.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprolab_ass2/MovingAction.h>
#include <cstdlib>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <exprolab_ass2/ErlOracle.h>
#include <exprolab_ass2/correct_hyp.h>
#include <string.h>
#include <std_msgs/String.h>



namespace KCL_rosplan {

    CheckcompleteInterface::CheckcompleteInterface(ros::NodeHandle &nh) {
    }

    bool CheckcompleteInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

    /**
    * \brief: CheckcompleteInterface callback
    * \param msg : the variables received from the plan dispatcher
    * 
    * \return boolean : true if the hypotesis is complete and consistent, otherwise false
    * 
    * This function calls the /check service to retrieve a boolean that indicates if the hypotesis is compete and consistent
    * (true if it is, false otherwise)
    */

        ros::NodeHandle n1;
        ros::ServiceClient complete_client = n1.serviceClient<exprolab_ass2::correct_hyp>("/check");
    
        bool complete;
        exprolab_ass2::correct_hyp req;
        req.request.t = true;
        complete_client.call(req);

        complete = req.response.hypotesis;

        if(complete)
        {
            ROS_INFO("something complete");
            return true;
        }
        else
        {
            //publish_hyp.publish(temp);
            ROS_INFO("nothing complete");
            return false;
        }
    }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "Checkcomplete_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    
    KCL_rosplan::CheckcompleteInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}