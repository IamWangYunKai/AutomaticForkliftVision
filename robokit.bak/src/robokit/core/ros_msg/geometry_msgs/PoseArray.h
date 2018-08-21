#ifndef _MSG_POSE_ARRAY_H_
#define _MSG_POSE_ARRAY_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include "robokit/core/ros_msg/geometry_msgs/Pose.h"
#include <vector>
namespace geometry_msgs {
	struct PoseArray {
		std_msgs::Header header;
		std::vector<Pose> poses;
	};
}
#endif