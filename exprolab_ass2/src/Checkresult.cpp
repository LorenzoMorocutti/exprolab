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



ros::ServiceClient complete_client;
int hyp_received[20];
int i=0;

namespace KCL_rosplan {
CheckresultInterface::CheckresultInterface(ros::NodeHandle &nh) {
// here the initialization
}



bool CheckresultInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action
    std_msgs::String temp;
    //std_msgs::String empty="";
    exprolab_ass2::correct_hyp req;
    req.request.t = true;
    complete_client.call(req);

    temp.data = req.response.hypotesis;

    if(temp.data.length()==0)
    {
        return false;
    }
    else
    {
        publish_hyp.publish(temp);
        return true;
    }

    return true;
}

void complete_clbk(const std_msgs::String msg)
{
    char *ptr;

    ptr=strtok(msg.data, "/");

    while(ptr != NULL)
    {
        find=0;
        temp = int(ptr);

        for(j=0, j<i, j++)
        {
            if(hyp_received[j] == temp)
            {
                find = 1;
            }

        }

        if(find==0)
        {
            hyp_received[i] = temp;
            i++;
        }

        hyp_received[i] = int(ptr);
    }
}


}



int main(int argc, char **argv) {
    ros::init(argc, argv, "Checkresult_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    
    ros::NodeHandle n1;
    ros::ServiceClient complete_client = n1.serviceClient<exprolab_ass2::correct_hyp>("/check");
    ros::Subscriber sub_hyp = nh.subscribe("/complete_hypotesis", 1000, complete_clbk);
    KCL_rosplan::CheckresultInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}