#ifndef _MSG_TRANSFORM_STAMPED_H_
#define _MSG_TRANSFORM_STAMPED_H_
#include "robokit/core/ros_msg/geometry_msgs/Transform.h"
#include "robokit/core/ros_msg/std_msgs/Header.h"
namespace geometry_msgs {
	struct TransformStamped
	{
	//This expresses a transform from coordinate frame header.frame_id
	//to the coordinate frame child_frame_id

	//This message is mostly used by the
	//tf package.
	//See its documentation for more information.
	std_msgs::Header header;
	std::string child_frame_id;// # the frame id of the child frame
	Transform transform;
	};
}
#endif