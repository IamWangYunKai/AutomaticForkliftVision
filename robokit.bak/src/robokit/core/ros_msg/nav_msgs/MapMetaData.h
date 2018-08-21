#ifndef _MAP_META_DATA_H_
#define _MAP_META_DATA_H_
#include <robokit/core/ros_msg/geometry_msgs/Pose.h>
#include <robokit/core/time_server/rbk_time.h>
namespace nav_msgs {
	struct MapMetaData
	{
		rbk::core::Time map_load_time;
		float resolution;
		int width;
		int height;
		geometry_msgs::Pose origin;
	};
}
#endif