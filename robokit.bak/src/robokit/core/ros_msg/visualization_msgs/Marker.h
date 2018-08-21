#ifndef _MARKER_H_
#define _MARKER_H_
#include "robokit/core/ros_msg/std_msgs/Header.h"
#include "robokit/core/ros_msg/std_msgs/ColorRGBA.h"
#include <string>
#include "robokit/core/ros_msg/geometry_msgs/Pose.h"
#include "robokit/core/ros_msg/geometry_msgs/Vector3.h"
#include <robokit/core/time_server/rbk_time.h>
namespace visualization_msgs {
	struct Marker
	{
		enum {
			ARROW = 0,
			CUBE = 1,
			SPHERE = 2,
			CYLINDER = 3,
			LINE_STRIP = 4,
			LINE_LIST = 5,
			CUBE_LIST = 6,
			SPHERE_LIST = 7,
			POINTS = 8,
			TEXT_VIEW_FACING = 9,
			MESH_RESOURCE = 10,
			TRIANGLE_LIST = 11,
			ADD = 0,
			MODIFY = 0,
			_DELETE = 2,
			DELETEALL = 3
		};

		std_msgs::Header header;
		std::string ns;
		int id;
		int type;
		int action;
		geometry_msgs::Pose pose;
		geometry_msgs::Vector3 scale;
		std_msgs::ColorRGBA color;
		rbk::core::Duration lifetime;
		bool frame_locked;
		std::vector<geometry_msgs::Point> points;
		std::vector<std_msgs::ColorRGBA> colors;
		std::string text;
		std::string mesh_resource;
		bool mesh_use_embedded_materials;
	};
}
#endif