#ifndef _MSG_TWIST_H_
#define _MSG_TWIST_H_
#include "robokit/core/ros_msg/geometry_msgs/Vector3.h"
namespace geometry_msgs {
	struct Twist
	{
		Vector3  linear;
		Vector3  angular;
	};
}
#endif