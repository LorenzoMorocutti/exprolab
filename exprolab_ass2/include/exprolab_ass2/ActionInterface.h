#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"

namespace KCL_rosplan {
class MovetoInterface: public RPActionInterface
{
private:
public:
/* constructor */
MovetoInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class MovehomeInterface: public RPActionInterface
{
private:
public:
/* constructor */
MovehomeInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class MoveawayInterface: public RPActionInterface
{
private:
public:
/* constructor */
MoveawayInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class CheckhintInterface: public RPActionInterface
{
private:
public:
/* constructor */
CheckhintInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class CheckcompleteInterface: public RPActionInterface
{
private:
public:
/* constructor */
CheckcompleteInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class CheckresultInterface: public RPActionInterface
{
private:
public:
/* constructor */
CheckresultInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};
}