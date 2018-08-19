#ifndef _MSG_PLOYGON_H_
#define _MSG_PLOYGON_H_
#include "robokit/core/ros_msg/geometry_msgs/Point32.h"
#include <vector>
namespace geometry_msgs {
	struct Polygon
	{
		std::vector<Point32> points;
	};
}
#endif