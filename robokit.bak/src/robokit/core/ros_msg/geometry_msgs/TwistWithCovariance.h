#ifndef _MSG_TWIST_WITH_COVARIANCE_H_
#define _MSG_TWIST_WITH_COVARIANCE_H_
#include "robokit/core/ros_msg/geometry_msgs/Twist.h"
namespace geometry_msgs {
	struct TwistWithCovariance
	{
		Twist twist;
		double covariance[36];
	};
}
#endif