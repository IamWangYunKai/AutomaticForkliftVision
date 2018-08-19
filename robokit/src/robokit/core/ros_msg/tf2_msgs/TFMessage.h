#ifndef _MSG_TFMESSAGE_H_
#define _MSG_TFMESSAGE_H_
#include "robokit/core/ros_msg/geometry_msgs/TransformStamped.h"
struct TFMessage
{
	std::vector<geometry_msgs::TransformStamped> transforms;
};
#endif