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
// here the initialization
}



bool CheckcompleteInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action
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