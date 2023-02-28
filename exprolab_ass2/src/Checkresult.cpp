/** @ package exprolab_ass2
* 
*  \file Checkresult.cpp
*  \brief implements the 'check_result' action
*
*  \author Lorenzo Morocutti
*  \version 1.0
*  \date 12/02/2023
*  \details
*   
*  Subscribes to: <BR>
*	/complete_hypotesis 
*   /print_result
*
*  Publishes to: <BR>
*	 None
*
*  Services: <BR>
*    None
* 
*   Client Services: <BR>
*   /oracle_solution
*    
*
*  Action Services: <BR>
*    None
*
*  Description: <BR>
*  This node implements the 'check_result' action to check if the complete hypotesis is the correct one 
*  and, in that case, print the solution.
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
    }
    
    bool CheckresultInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

    /**
    * \brief: CheckresultInterface callback
    * \param msg : the variables received from the plan dispatcher
    * 
    * \return boolean : true if the hypotesis is the winning one, otherwise false
    * 
    * This function calls the /oracle_solution service to retrieve the winning ID and the /print_result service to 
    * retrieve the fields of the hypotesis (if it's the winning one)
    */

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

void complete_clbk(const std_msgs::Int32 msg){

    /**
    * \brief: Callback for the subscriber to /complete_hypotesis
    * \param msg : the variables received from the publisher
    * 
    * \return : None
    * 
    * This function receives the complete hypotesis and saves their IDs in an array (if I didn't saved them already)
    */

    //ROS_INFO("SONO DENTRO A RESULT");

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