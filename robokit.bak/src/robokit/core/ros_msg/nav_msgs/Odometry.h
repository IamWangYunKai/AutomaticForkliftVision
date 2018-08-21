#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include "robokit/core/ros_msg/geometry_msgs/PoseStamped.h"
#include "robokit/core/ros_msg/geometry_msgs/PoseWithCovariance.h"
#include "robokit/core/ros_msg/geometry_msgs/TwistWithCovariance.h"
#include <string>
namespace nav_msgs {
	struct Odometry
	{
		std_msgs::Header header;
		std::string child_frame_id;
		geometry_msgs::PoseWithCovariance pose;
		geometry_msgs::TwistWithCovariance twist;
	};
}
#endif