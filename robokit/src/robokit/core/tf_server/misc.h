#ifndef _MISC_H_
#define _MISC_H_
#include <robokit/core/logger.h>
#include <robokit/core/time_server/rbk_time.h>
#define ROS_WARN LogWarn
#define ROS_ERROR LogError
#define ROS_DEBUG LogInfo
#define ROS_INFO LogInfo
#define ROS_ASSERT BOOST_ASSERT
#define ROS_ASSERT_MSG BOOST_ASSERT_MSG
#define ROS_INFO_STREAM LogInfo
#define ROS_ERROR_STREAM LogError
#define ROS_WARN_STREAM LogWarn
//#define ros rbk::core
#endif