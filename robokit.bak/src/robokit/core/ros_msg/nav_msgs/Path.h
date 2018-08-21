#ifndef _PATH_H_
#define _PATH_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include "robokit/core/ros_msg/geometry_msgs/PoseStamped.h"
namespace nav_msgs {
	struct Path
	{
		std_msgs::Header header;
		std::vector<geometry_msgs::PoseStamped> poses;
	};
}
#endif