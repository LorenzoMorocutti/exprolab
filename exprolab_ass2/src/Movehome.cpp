#include "exprolab_ass2/ActionInterface.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprolab_ass2/MovingAction.h>

namespace KCL_rosplan {
MovehomeInterface::MovehomeInterface(ros::NodeHandle &nh) {
// here the initialization

}

bool MovehomeInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action
    ros::NodeHandle nh("~");
    actionlib::SimpleActionClient<exprolab_ass2::MovingAction> action_client("/go_to_point", true);
    //std::cout << "Going from " << msg-->parameters[0].value << " to " << msg -->parameters[1].value << std::endl
    exprolab_ass2::MovingGoal goal;
    action_client.waitForServer();
    
    if(msg->parameters[1].value == "start")
    {
        goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
    }

    action_client.sendGoal(goal);
    action_client.waitForResult();


    //ROS_INFO("Action (%s) performed: completed!", msg-->name.c_);
    return true;
}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Movehome_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
   // ros::ServiceClient client_p = nh.serviceClient<exprolab_ass2::Position>("/go_to_point");
    KCL_rosplan::MovehomeInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}