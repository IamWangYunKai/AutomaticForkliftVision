#ifndef _ROS_SENSOR_MSGS_POINT_FIELD_H_
#define _ROS_SENSOR_MSGS_POINT_FIELD_H_
#include <string>
namespace sensor_msgs {
	struct PointField
	{
		enum 
		{
			 INT8 = 1,
			 UINT8 = 2,
			 INT16 = 3,
			 UINT16 = 4,
			 INT32 = 5,
			 UINT32 = 6,
			 FLOAT32 = 7,
			 FLOAT64 = 8
		};
		std::string name;
		int offset;
		int datatype;
		int count;
	};
}
#endif