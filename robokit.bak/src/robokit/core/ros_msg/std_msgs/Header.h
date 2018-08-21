#ifndef _HEADER_H_
#define _HEADER_H_
#include <string>
#include <robokit/core/time_server/rbk_time.h>
namespace std_msgs {
	struct Header {
		int seq;
		rbk::core::Time stamp;
		std::string frame_id;
	};
}
#endif