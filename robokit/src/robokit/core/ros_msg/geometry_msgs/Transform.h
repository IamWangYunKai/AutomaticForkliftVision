#ifndef _MSG_TRANSFORM_H_
#define _MSG_TRANSFORM_H_
#include "robokit/core/ros_msg/geometry_msgs/Vector3.h"
#include "robokit/core/ros_msg/geometry_msgs/Quaternion.h"
namespace geometry_msgs {
	struct Transform
	{
		Vector3 translation;
		Quaternion rotation;
	};
}
#endif