#ifndef _MSG_POSE_STAMPED_H_
#define _MSG_POSE_STAMPED_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include "robokit/core/ros_msg/geometry_msgs/Pose.h"
namespace geometry_msgs {
	struct PoseStamped {
		std_msgs::Header header;
		Pose pose;
	};
}
#endif