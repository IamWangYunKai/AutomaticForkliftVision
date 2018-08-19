#ifndef _MSG_TWIST_STAMPED_H_
#define _MSG_TWIST_STAMPED_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include "robokit/core/ros_msg/geometry_msgs/Twist.h"
namespace geometry_msgs{
	struct TwistStamped
	{
		std_msgs::Header header;
		Twist twist;
	};
}
#endif