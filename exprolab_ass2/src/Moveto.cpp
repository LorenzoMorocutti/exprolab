/** @ package exprolab_ass2
* 
*  \file Moveto.cpp
*  \brief implements the 'move_to' action
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
*  This node implements the 'move_to' action to a waypoint. This is possible thanks to 
*  the go_to_point node.
*/

#include "exprolab_ass2/ActionInterface.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprolab_ass2/MovingAction.h>

namespace KCL_rosplan {
    MovetoInterface::MovetoInterface(ros::NodeHandle &nh) {
    }

    bool MovetoInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
    
    /**
    * \brief: MovetoInterface callback
    * \param msg : the variables received from the plan dispatcher
    * 
    * \return true
    * 
    * This function calls the action client to communicate with go_to_point and, depending on which
    * waypoint the robot has to reach, set different goal positions
    */

        ros::NodeHandle nh("~");
        actionlib::SimpleActionClient<exprolab_ass2::MovingAction> action_client("/go_to_point", true);

        exprolab_ass2::MovingGoal goal;
        action_client.waitForServer();

        // position of w1
        if(msg->parameters[1].value == "w1")
        {
            goal.target_pose.pose.position.x = 2.5;
		    goal.target_pose.pose.position.y = 0.0;
		    goal.target_pose.pose.position.z = -3.14/2;
        }
        // position of w2
        else if(msg->parameters[1].value == "w2")
        {
            goal.target_pose.pose.position.x = 0.0;
		    goal.target_pose.pose.position.y = 2.5;
		    goal.target_pose.pose.position.z = 0.0;
        }
        // position of w3
        else if(msg->parameters[1].value == "w3")
        {
            goal.target_pose.pose.position.x = -2.5;
		    goal.target_pose.pose.position.y = 0.0;
		    goal.target_pose.pose.position.z = 3.14/2;
        }
        // position of w4
        else if(msg->parameters[1].value == "w4")
        {
            goal.target_pose.pose.position.x = 0.0;
		    goal.target_pose.pose.position.y = -2.5;
		    goal.target_pose.pose.position.z = 3.14;
        }

        action_client.sendGoal(goal);
        action_client.waitForResult();

        return true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Moveto_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
   // ros::ServiceClient client_p = nh.serviceClient<exprolab_ass2::Position>("/go_to_point");
    KCL_rosplan::MovetoInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}