#include "exprolab_ass2/ActionInterface.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprolab_ass2/MovingAction.h>
#include <cstdlib>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <exprolab_ass2/ErlOracle.h>
#include <exprolab_ass2/hint.h>
#include <string.h>


bool high_pose = false; 
bool received = false;
ros::ServiceClient hint_client;

void move_arm();
void receive_clbk(const exprolab_ass2::ErlOracle msg);


namespace KCL_rosplan {
CheckhintInterface::CheckhintInterface(ros::NodeHandle &nh) {
// here the initialization
}



bool CheckhintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action

    move_arm();

    return true;
}
}

void receive_clbk(const exprolab_ass2::ErlOracle msg)
{
    
    ros::NodeHandle n1;
    ros::ServiceClient hint_client = n1.serviceClient<exprolab_ass2::hint>("/hint");
    exprolab_ass2::hint req;
    req.request.ID = msg.ID;
    req.request.key = msg.key;
    req.request.value = msg.value;
    ROS_INFO("dovrei mandaee request");
    hint_client.call(req);
    received = true;
}

void move_arm()
{   
    if(high_pose)
    {
        if(received == false)
        {
            geometry_msgs::Pose pose1;
	        moveit::planning_interface::MoveGroupInterface group("arm");
	        group.setEndEffectorLink("cluedo_link");
	        group.setPoseReferenceFrame("arm_base_link");
	        group.setPlannerId("RRTstar");
	        group.setNumPlanningAttempts(10);
	        group.setPlanningTime(10.0);
	        group.allowReplanning(true);
	        group.setGoalPositionTolerance(0.05);
	        pose1.position.x = 0.0;
	        pose1.position.y = 0.5;
	        pose1.position.z = 0.75;
	        group.setStartStateToCurrentState();
	        group.setApproximateJointValueTarget(pose1, "cluedo_link");
	        moveit::planning_interface::MoveGroupInterface::Plan plan;
	        group.plan(plan);
	        group.execute(plan);
            high_pose = false;
        }
        else
        {
            received = false;
        }
    }
    else
    {
        if(received == false)
        {
            geometry_msgs::Pose pose1;
	        moveit::planning_interface::MoveGroupInterface group("arm");
	        group.setEndEffectorLink("cluedo_link");
	        group.setPoseReferenceFrame("arm_base_link");
	        group.setPlannerId("RRTstar");
	        group.setNumPlanningAttempts(10);
	        group.setPlanningTime(10.0);
	        group.allowReplanning(true);
	        group.setGoalPositionTolerance(0.05);
	        pose1.position.x = 0.0;
	        pose1.position.y = 0.5;
	        pose1.position.z = 1.25;
	        group.setStartStateToCurrentState();
	        group.setApproximateJointValueTarget(pose1, "cluedo_link");
	        moveit::planning_interface::MoveGroupInterface::Plan plan;
	        group.plan(plan);
	        group.execute(plan);
            high_pose = true;
        }
        else
        {
            received = false;
        }
    }
    received=false;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "Checkhint_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    ros::Subscriber receive_hint = nh.subscribe("/oracle_hint", 1000, receive_clbk);
    
    KCL_rosplan::CheckhintInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}