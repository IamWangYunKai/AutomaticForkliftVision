#ifndef _ROS_SENSOR_MSGS_POINT_CLOUD2_H_
#define _ROS_SENSOR_MSGS_POINT_CLOUD2_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include "robokit/core/ros_msg/sensor_msgs/PointField.h"
#include <vector>
namespace sensor_msgs {
	struct PointCloud2
	{
		std_msgs::Header header;
		int height;
		int width;
		std::vector<sensor_msgs::PointField> fields;
		bool is_bigendian;
		int point_step;
		int row_step;
		std::vector<unsigned char> data;
		bool is_dense;
	};
}
#endif