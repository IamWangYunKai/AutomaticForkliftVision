#ifndef _MSG_POLYGON_STAMPED_H_
#define _MSG_POLYGON_STAMPED_H_
#include "robokit/core/ros_msg/geometry_msgs/Polygon.h"
#include "robokit/core/ros_msg/std_msgs/Header.h"
namespace geometry_msgs {
	struct PolygonStamped
	{
		std_msgs::Header header;
		Polygon polygon;
	};
}
#endif