#ifndef _MSG_POSE_WITH_COVARIANCE_H_
#define _MSG_POSE_WITH_COVARIANCE_H_
#include "robokit/core/ros_msg/geometry_msgs/Pose.h"
namespace geometry_msgs
{
	struct PoseWithCovariance
	{
		Pose pose;
		double covariance[36];
	};
}
#endif