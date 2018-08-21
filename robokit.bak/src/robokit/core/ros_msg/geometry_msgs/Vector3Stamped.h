#ifndef _MSG_VECTOR3_STAMPED_H_
#define _MSG_VECTOR3_STAMPED_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include "robokit/core/ros_msg/geometry_msgs/Vector3.h"
namespace geometry_msgs {
	struct Vector3Stamped {
		std_msgs::Header header;
		Vector3 vector;
	};
}
#endif