#ifndef _ROS_SENSOR_MSG_LASER_SCAN_H_
#define _ROS_SENSOR_MSG_LASER_SCAN_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include <vector>
namespace sensor_msgs {
	struct LaserScan
	{
		std_msgs::Header header;
		double angle_min;
		double angle_max;
		double angle_increment;
		double time_increment;
		double scan_time;
		double range_min;
		double range_max;
		
		std::vector<double> ranges;
		std::vector<double> intensities;
		std::vector<bool> valid;
	};
}
#endif