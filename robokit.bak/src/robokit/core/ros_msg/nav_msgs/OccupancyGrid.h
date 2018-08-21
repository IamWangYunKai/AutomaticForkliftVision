#ifndef _OCCUPANCY_GRID_H_
#define _OCCUPANCY_GRID_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include <robokit/core/ros_msg/nav_msgs/MapMetaData.h>
namespace nav_msgs {
	struct OccupancyGrid
	{
		std_msgs::Header header;
		nav_msgs::MapMetaData info;
		std::vector<unsigned char> data;
	};
}
#endif