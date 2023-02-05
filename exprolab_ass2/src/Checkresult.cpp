#include "exprolab_ass2/ActionInterface.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprolab_ass2/MovingAction.h>
#include <cstdlib>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <exprolab_ass2/ErlOracle.h>
#include <exprolab_ass2/Oracle.h>
#include <exprolab_ass2/correct_hyp.h>
#include <string.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <exprolab_ass2/print_res.h>

int hyp_received[20];
int count=0;
void complete_clbk(const std_msgs::Int32 msg);

namespace KCL_rosplan {
CheckresultInterface::CheckresultInterface(ros::NodeHandle &nh) {
// here the initialization
}



bool CheckresultInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action
    ros::NodeHandle n1;
    ros::ServiceClient correct_client = n1.serviceClient<exprolab_ass2::Oracle>("/oracle_solution");
    ros::ServiceClient print_client = n1.serviceClient<exprolab_ass2::print_res>("/print_result");
    exprolab_ass2::Oracle req;
    correct_client.call(req);
    int id_win = req.response.ID;
    std::cout << id_win << std::endl;
    int i;
    for(i=0; i<count; i++)
    {
        std::cout << "element"<< hyp_received[i] << std::endl; 
        if(int(hyp_received[i]) == id_win)
        {
            
            exprolab_ass2::print_res req_print;
            req_print.request.ID=id_win;
            print_client.call(req_print);
            std::cout << "The killer was " << req_print.response.who << " in the " << req_print.response.where << " with the " << req_print.response.what <<std::endl; 
            return true;
        }       
    }

    return false;
}




}

void complete_clbk(const std_msgs::Int32 msg)
{
    ROS_INFO("SONO DENTRO A RESULT");

    bool found = false;
    int i;
    for(i=0; i<count; i++)
    {
        if(hyp_received[i] == msg.data)
        {
            found = true;
            std::cout <<"dentro found"<<std::endl;
        }
    }
    if(found == false)
    {
        std::cout <<"dentro if giiudto"<<std::endl;
        hyp_received[count] = msg.data;
        count++;
    }
    std::cout << "count" << count <<std::endl;
    std::cout << "hyp_received" << hyp_received[count-1] <<std::endl;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Checkresult_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    
    ros::Subscriber sub_hyp=nh.subscribe("/complete_hypotesis", 1000, complete_clbk);
    KCL_rosplan::CheckresultInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}