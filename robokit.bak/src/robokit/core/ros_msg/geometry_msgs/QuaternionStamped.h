#ifndef _MSG_QUATERNION_STAMPED_H_
#define _MSG_QUATERNION_STAMPED_H_
#include "robokit/core/ros_msg/geometry_msgs/Quaternion.h"
#include "robokit/core/ros_msg/std_msgs/Header.h"
namespace geometry_msgs {
	struct QuaternionStamped
	{
		std_msgs::Header header;
		Quaternion quaternion;
	};
}

#endif