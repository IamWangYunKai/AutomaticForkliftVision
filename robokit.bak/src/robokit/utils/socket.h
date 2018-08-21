#ifndef _RBK_UTILS_SOCKET_H_
#define _RBK_UTILS_SOCKET_H_

#include <robokit/config.h>
#include <cstdint>
#include <string>
#include <vector>
#include <boost/system/error_code.hpp>

namespace rbk {
    namespace utils {
        namespace socket {

            const int bufferSize = 102400;

            RBK_EXTERN RBK_API uint32_t networkToHostLong(uint32_t value);

            RBK_EXTERN RBK_API uint32_t hostToNetworkLong(uint32_t value);

            RBK_EXTERN RBK_API uint16_t networkToHostShort(uint16_t value);

            RBK_EXTERN RBK_API uint16_t hostToNetworkShort(uint16_t value);

            RBK_EXTERN RBK_API void setKeepAliveParam(int socket, bool on, uint32_t idle, uint32_t interval, uint32_t count);

            RBK_EXTERN RBK_API void localIP(std::vector<std::string>& ips, boost::system::error_code& ec);
        } // namespace socket
    } // namespace utils
} // namespace rbk

#endif // ~_RBK_UTILS_SOCKET_H_
