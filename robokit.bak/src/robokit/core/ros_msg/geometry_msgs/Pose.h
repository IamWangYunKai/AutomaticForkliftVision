#ifndef _MSG_POSE_H_
#define _MSG_POSE_H_
#include "robokit/core/ros_msg/geometry_msgs/Point.h"
#include "robokit/core/ros_msg/geometry_msgs/Quaternion.h"
namespace geometry_msgs {
	struct Pose
	{
		Point position;
		Quaternion orientation;
	};
}
#endif