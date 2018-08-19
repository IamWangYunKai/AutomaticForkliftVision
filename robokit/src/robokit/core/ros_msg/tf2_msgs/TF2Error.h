#ifndef _MSG_TF2ERROR_H_
#define _MSG_TF2ERROR_H_
#include <string>
namespace tf2_msgs {
	struct TF2Error
	{
		enum
		{
			_NO_ERROR = 0,
			LOOKUP_ERROR = 1,
			CONNECTIVITY_ERROR = 2,
			EXTRAPOLATION_ERROR = 3,
			INVALID_ARGUMENT_ERROR = 4,
			TIMEOUT_ERROR = 5,
			TRANSFORM_ERROR = 6
		};


		int error;
		std::string error_string;
	};
}

#endif