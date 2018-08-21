#ifndef _MSG_MARKER_H_
#define _MSG_MARKER_H_
#include "robokit/core/ros_msg/visualization_msgs/Marker.h"
namespace visualization_msgs {
	struct MarkerArray
	{
		std::vector<visualization_msgs::Marker> markers;
	};
}
#endif