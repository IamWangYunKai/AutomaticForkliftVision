#ifndef _ROS_SENSOR_MSGS_POINT_CLOUD_H_
#define _ROS_SENSOR_MSGS_POINT_CLOUD_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include "robokit/core/ros_msg/geometry_msgs/Point32.h"
#include "robokit/core/ros_msg/sensor_msgs/ChannelFloat32.h"
#include <vector>
namespace sensor_msgs {
	struct PointCloud
	{
		std_msgs::Header header;
		std::vector<geometry_msgs::Point32> points;
		std::vector<sensor_msgs::ChannelFloat32> channels;
	};
}
#endif