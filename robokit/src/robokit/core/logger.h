#ifndef _RBK_LOGGER_H_
#define _RBK_LOGGER_H_

#include <robokit/config.h>
#include <robokit/core/lua_wrapper.h>

#include <boost/log/trivial.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#include <boost/log/attributes/mutable_constant.hpp>
#include <boost/log/utility/manipulators/add_value.hpp>
#include <string>

#ifdef RBK_SYS_WINDOWS
#include <windows.h>
#endif // RBK_SYS_WINDOWS


#if defined(RBK_SYS_WINDOWS)
BOOST_LOG_GLOBAL_LOGGER(RBK_API coreLogger,
    boost::log::sources::severity_channel_logger_mt<boost::log::trivial::severity_level>);
BOOST_LOG_GLOBAL_LOGGER(RBK_API debugLogger,
    boost::log::sources::severity_channel_logger_mt<boost::log::trivial::severity_level>);
#else
BOOST_LOG_GLOBAL_LOGGER(coreLogger,
    boost::log::sources::severity_channel_logger_mt<boost::log::trivial::severity_level>);
BOOST_LOG_GLOBAL_LOGGER(debugLogger,
    boost::log::sources::severity_channel_logger_mt<boost::log::trivial::severity_level>);
#endif

template <typename T>
std::string arg2String(const T& arg) {
    return std::to_string(arg);
}

template <typename T>
std::string arg2String(const rbk::lua::Nil<std::vector<T>>& arg) {
    std::stringstream s;
    s << arg;
    return s.str();
}

template <typename T>
std::string arg2String(const rbk::lua::Nil<T>& arg) {
    std::stringstream s;
    s << arg;
    return s.str();
}

template <>
RBK_API std::string arg2String(const std::string& arg);

template <>
RBK_API std::string arg2String(const bool& arg);


RBK_API std::string arg2String(const char* arg);

template<typename... Args>
std::string formatLog(const std::string& type, const Args&... args) {
    std::string str[] = { arg2String(args)... };
    std::string fStr = "[";
    fStr.append(type).append("][");
    for (int i = 0; i < sizeof...(args); ++i) {
        if (i != (sizeof...(args)) - 1) {
            fStr.append(str[i]).append("|");
        }
        else {
            fStr.append(str[i]);
        }
    }
    return fStr.append("]");
}

#define formatLogText(ARG) \
    "[Text][" << ARG << "]";


namespace rbk {
    class RBK_API Logger {
    public:
        /// Init with default trivial logging
        static void init();

        /// @param configFileName config ini file that contains boost logging properties.
        ///        If configFileName.empty() then default initialization.
        static void initFromConfig(const std::string& configFileName);

        /// Disable logging
        static void disable();

        /// Add a file sink for LOG_DATA_* for >= INFO.
        /// This file sink will be used along with any configured via Config in init().
        static void addDataFileLog(const std::string& logFileName);

        // solution for crashes on process termination when file sinks are used?
        // see http://www.boost.org/doc/libs/1_60_0/libs/log/doc/html/log/rationale/why_crash_on_term.html
        static void destroy();
    };
} // namespace rbk

#if defined(__RBK_PLUGIN__)

//#define LOG_LOG_RECORD(LOGGER, TYPE, ARG)               \
//    BOOST_LOG_SEV(LOGGER, boost::log::trivial::debug)   \
//    << boost::log::add_value("Plugin", __RBK_PLUGIN__)  \
//    << boost::log::add_value("Line", __LINE__)          \
//    << boost::log::add_value("File", __FILE__)          \
//    << boost::log::add_value("Function", TYPE) << ARG;

#define LOG_LOG_RECORD(LOGGER, TYPE, ARG)               \
    BOOST_LOG_SEV(LOGGER, boost::log::trivial::debug)   \
    << boost::log::add_value("Plugin", __RBK_PLUGIN__)  \
    << boost::log::add_value("Function", TYPE) << ARG;

#define LOG_LOG_LOCATION(LOGGER, LEVEL, ARG)            \
  BOOST_LOG_SEV(LOGGER, boost::log::trivial::LEVEL)     \
    << boost::log::add_value("Plugin", __RBK_PLUGIN__)  \
    << boost::log::add_value("Function", __FUNCTION__) << ARG;

#if defined(RBK_SYS_WINDOWS) && RBK_WINVER < 0x0A00

#define LOG_LOG_WARN_LOCATION(LOGGER, LEVEL, ARG)            \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY); \
    BOOST_LOG_SEV(LOGGER, boost::log::trivial::LEVEL)     \
    << boost::log::add_value("Plugin", __RBK_PLUGIN__)  \
    << boost::log::add_value("Function", __FUNCTION__) << ARG; \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);

#define LOG_LOG_ERROR_LOCATION(LOGGER, LEVEL, ARG)            \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_INTENSITY); \
    BOOST_LOG_SEV(LOGGER, boost::log::trivial::LEVEL)     \
    << boost::log::add_value("Plugin", __RBK_PLUGIN__)  \
    << boost::log::add_value("Function", __FUNCTION__) << ARG; \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);

#define LOG_LOG_FATAL_LOCATION(LOGGER, LEVEL, ARG)            \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_INTENSITY); \
    BOOST_LOG_SEV(LOGGER, boost::log::trivial::LEVEL)     \
    << boost::log::add_value("Plugin", __RBK_PLUGIN__)  \
    << boost::log::add_value("Function", __FUNCTION__) << ARG; \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);

#endif // RBK_SYS_WINDOWS

#else

#define LOG_LOG_RECORD(LOGGER, TYPE, ARG)               \
    BOOST_LOG_SEV(LOGGER, boost::log::trivial::debug)   \
    << boost::log::add_value("Plugin", "Robokit")       \
    << boost::log::add_value("Function", TYPE) << ARG;

#define LOG_LOG_LOCATION(LOGGER, LEVEL, ARG)            \
    BOOST_LOG_SEV(LOGGER, boost::log::trivial::LEVEL)   \
    << boost::log::add_value("Plugin", "Robokit")       \
    << boost::log::add_value("Function", __FUNCTION__) << ARG;

#if defined(RBK_SYS_WINDOWS) && RBK_WINVER < 0x0A00

#define LOG_LOG_WARN_LOCATION(LOGGER, LEVEL, ARG)            \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY); \
    BOOST_LOG_SEV(LOGGER, boost::log::trivial::LEVEL)     \
    << boost::log::add_value("Plugin", "Robokit")       \
    << boost::log::add_value("Function", __FUNCTION__) << ARG; \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);

#define LOG_LOG_ERROR_LOCATION(LOGGER, LEVEL, ARG)            \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_INTENSITY); \
    BOOST_LOG_SEV(LOGGER, boost::log::trivial::LEVEL)     \
    << boost::log::add_value("Plugin", "Robokit")       \
    << boost::log::add_value("Function", __FUNCTION__) << ARG; \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);

#define LOG_LOG_FATAL_LOCATION(LOGGER, LEVEL, ARG)            \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_INTENSITY); \
    BOOST_LOG_SEV(LOGGER, boost::log::trivial::LEVEL)     \
    << boost::log::add_value("Plugin", "Robokit")       \
    << boost::log::add_value("Function", __FUNCTION__) << ARG; \
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);

#endif // RBK_SYS_WINDOWS
#endif

/// System Log macros.
/// TRACE < DEBUG < INFO < WARN < ERROR < FATAL
#define LogRecord(TYPE, ARG) LOG_LOG_RECORD(coreLogger::get(), TYPE, ARG);
#define LogTrace(ARG) LOG_LOG_LOCATION(coreLogger::get(), trace, ARG);
#define LogDebug(ARG) LOG_LOG_LOCATION(coreLogger::get(), debug, ARG);
#define LogInfo(ARG)  LOG_LOG_LOCATION(coreLogger::get(), info, ARG);
#if defined(RBK_SYS_WINDOWS) && RBK_WINVER < 0x0A00
#define LogWarn(ARG)  {LOG_LOG_WARN_LOCATION(coreLogger::get(), warning, ARG);}
#define LogError(ARG) {LOG_LOG_ERROR_LOCATION(coreLogger::get(), error, ARG);}
#define LogFatal(ARG) {LOG_LOG_FATAL_LOCATION(coreLogger::get(), fatal, ARG);}
#else
#define LogWarn(ARG)  LOG_LOG_LOCATION(coreLogger::get(), warning, ARG);
#define LogError(ARG) LOG_LOG_LOCATION(coreLogger::get(), error, ARG);
#define LogFatal(ARG) LOG_LOG_LOCATION(coreLogger::get(), fatal, ARG);
#endif // RBK_SYS_WINDOWS && RBK_WINVER < 0x0A00

/// Debug Log macros.
/// TRACE < DEBUG < INFO < WARN < ERROR < FATAL
#define LogTraceD(ARG) LOG_LOG_LOCATION(debugLogger::get(), trace, ARG)
#define LogDebugD(ARG) LOG_LOG_LOCATION(debugLogger::get(), debug, ARG)
#define LogInfoD(ARG)  LOG_LOG_LOCATION(debugLogger::get(), info, ARG)
#if defined(RBK_SYS_WINDOWS) && RBK_WINVER < 0x0A00
#define LogWarnD(ARG)  {LOG_LOG_WARN_LOCATION(debugLogger::get(), warning, ARG);}
#define LogErrorD(ARG) {LOG_LOG_ERROR_LOCATION(debugLogger::get(), error, ARG);}
#define LogFatalD(ARG) {LOG_LOG_FATAL_LOCATION(debugLogger::get(), fatal, ARG);}
#else
#define LogWarnD(ARG)  LOG_LOG_LOCATION(debugLogger::get(), warning, ARG)
#define LogErrorD(ARG) LOG_LOG_LOCATION(debugLogger::get(), error, ARG)
#define LogFatalD(ARG) LOG_LOG_LOCATION(debugLogger::get(), fatal, ARG)
#endif // RBK_SYS_WINDOWS && RBK_WINVER < 0x0A00

#endif //~_RBK_LOGGER_H_
