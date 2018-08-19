#ifndef _MSG_POINT_STAMPED_H_
#define _MSG_POINT_STAMPED_H_
#include "robokit/core/ros_msg/geometry_msgs/Point.h"
#include "robokit/core/ros_msg/std_msgs/Header.h"
namespace geometry_msgs {
	struct PointStamped {
		std_msgs::Header header;
		Point point;
	};
}
#endif