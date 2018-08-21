#ifndef _RBK_UTILS_CODE_BASE64_H_
#define _RBK_UTILS_CODE_BASE64_H_

#include <robokit/config.h>

#include <string>
#include <ctype.h>

namespace rbk {
    namespace utils {
        namespace code {
            namespace base64 {
                // 是否为 base64 字符
                static inline bool isBase64(unsigned char c) {
                    return (isalnum(c) || (c == '+') || (c == '/'));
                }

                RBK_API std::string encode(const unsigned char* bytesToEncode, unsigned int len);

                RBK_API std::string encode(const std::string& str);

                RBK_API std::string decode(const std::string& encodedString);
            }
        }
    }
}

#endif // _RBK_UTILS_SERIAL_PORT_H_
