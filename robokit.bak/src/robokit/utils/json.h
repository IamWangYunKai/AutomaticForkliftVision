#ifndef _RBK_UTILS_JSON_H_
#define _RBK_UTILS_JSON_H_

#include <robokit/config.h>

#include <json/json.hpp>

namespace rbk {
    namespace utils {
        using json = nlohmann::json;
    } // namespace utils
} // namespace rbk

#endif // ~_RBK_UTILS_JSON_H_
