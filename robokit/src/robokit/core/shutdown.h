#ifndef _RBK_CORE_SHUTDOWN_H_
#define _RBK_CORE_SHUTDOWN_H_

#include <robokit/config.h>

namespace rbk {
    namespace core {
        RBK_API bool logoff();

        // need sudo under linux and macOS
        RBK_API bool reboot();

        // need sudo under linux and macOS
        RBK_API bool shutdown();
    } // namespace core
} // namespace rbk

#endif // ~_RBK_CORE_SHUTDOWN_H_
