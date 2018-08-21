#ifndef _ROS_SENSOR_MSGS_CHANNEL_FLOAT32_H_
#define _ROS_SENSOR_MSGS_CHANNEL_FLOAT32_H_
#include <string>
#include <vector>
namespace sensor_msgs {
	struct ChannelFloat32
	{
		std::string name;
		std::vector<float> values;
	};
}
#endif