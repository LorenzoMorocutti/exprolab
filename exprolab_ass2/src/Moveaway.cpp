#include "exprolab_ass2/ActionInterface.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprolab_ass2/MovingAction.h>

namespace KCL_rosplan {
MoveawayInterface::MoveawayInterface(ros::NodeHandle &nh) {
// here the initialization

}

bool MoveawayInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action
ROS_INFO("IN HERE");
    actionlib::SimpleActionClient<exprolab_ass2::MovingAction> action_client("/go_to_point", true);
    //std::cout << "Going from " << msg-->parameters[0].value << " to " << msg -->parameters[1].value << std::endl
    exprolab_ass2::MovingGoal goal;
    action_client.waitForServer();
    ROS_INFO("SERVER LOADED");
    if(msg->parameters[1].value == "w1")
    {
        goal.target_pose.pose.position.x = 2.5;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.position.z = -3.14/2;
    }
    else if(msg->parameters[1].value == "w2")
    {
        goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 2.5;
		goal.target_pose.pose.position.z = 0.0;
    }
    else if(msg->parameters[1].value == "w3")
    {
        goal.target_pose.pose.position.x = -2.5;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.position.z = 3.14/2;
    }
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
    ros::init(argc, argv, "Moveaway_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
   // ros::ServiceClient client_p = nh.serviceClient<exprolab_ass2::Position>("/go_to_point");
    KCL_rosplan::MoveawayInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}