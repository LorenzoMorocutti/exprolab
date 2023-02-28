/** @ package exprolab_ass2
* 
*  \file Movehome.cpp
*  \brief implements the 'move_home' action
*
*  \author Lorenzo Morocutti
*  \version 1.0
*  \date 12/02/2023
*  \details
*   
*  Subscribes to: <BR>
*	None 
*
*  Publishes to: <BR>
*	 None
*
*  Services: <BR>
*    None
* 
*   Client Services: <BR>
*   None
*    
*
*  Action Services: <BR>
*    go_to_point
*
*  Description: <BR>
*  This node implements the 'move_home' action to return to the home waypoint. This is possible thanks to 
*  the go_to_point node.
*/

#include "exprolab_ass2/ActionInterface.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprolab_ass2/MovingAction.h>

namespace KCL_rosplan {
    MovehomeInterface::MovehomeInterface(ros::NodeHandle &nh) {
    }

    bool MovehomeInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
    
    /**
    * \brief: MovehomeInterface callback
    * \param msg : the variables received from the plan dispatcher
    * 
    * \return true
    * 
    * This function calls the action client to communicate with go_to_point and set the home waypoint as goal position
    */

        ros::NodeHandle nh("~");
        actionlib::SimpleActionClient<exprolab_ass2::MovingAction> action_client("/go_to_point", true);
        exprolab_ass2::MovingGoal goal;
        action_client.waitForServer();
    
        // set the home position
        if(msg->parameters[1].value == "start")
        {
            goal.target_pose.pose.position.x = 0.0;
		    goal.target_pose.pose.position.y = 0.0;
		    goal.target_pose.pose.orientation.w = 0.0;
        }

        action_client.sendGoal(goal);
        action_client.waitForResult();

        return true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Movehome_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    KCL_rosplan::MovehomeInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}