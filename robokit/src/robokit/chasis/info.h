#ifndef _RBK_CHASIS_INFO_H_
#define _RBK_CHASIS_INFO_H_

#include <robokit/config.h>
#include <robokit/utils/json.h>
#include <robokit/core/mutex.h>

namespace rbk {
    namespace chasis {

        class RBK_API Info {
        private:
            struct object_creator {
                // This constructor does nothing more than ensure that Instance()
                // is called before main() begins, thus creating the static
                // SingletonClass object before multithreading race issues can come up.
                object_creator() { Info::Instance(); }
                inline void do_nothing() const { }
            };

            static object_creator create_object;

            Info();

            ~Info();

            Info(const Info&);

            Info& operator=(const Info&);

        public:

            static Info* Instance();

            template<typename T>
            bool set(const std::string& pointer, const T& value, std::string& errMsg = std::string("")) {
                try {
                    rbk::utils::json::json_pointer p(pointer);
                    rbk::writeLock l(_mutex);
                    _jsonDoc[p] = value;
                    return true;
                }
                catch (const std::exception& e) {
                    errMsg = e.what();
                    return false;
                }
            }

            rbk::utils::json getJson();

            template<typename T>
            bool get(const std::string& pointer, T& value, std::string& errMsg = std::string("")) {
                try {
                    rbk::utils::json::json_pointer p(pointer);
                    rbk::readLock l(_mutex);
                    T t = _jsonDoc[p];
                    value = t;
                    return true;
                }
                catch (const std::exception& e) {
                    errMsg = e.what();
                    return false;
                }
            }

            template<typename T>
            T get(const std::string& pointer) {
                try {
                    rbk::utils::json::json_pointer p(pointer);
                    rbk::readLock l(_mutex);
                    T t = _jsonDoc[p];
                    return t;
                }
                catch (const std::exception&) {
                    return T();
                }
            }

        private:

            rbk::utils::json _jsonDoc;

            rbk::rwMutex _mutex;

        };
    } // namespace chasis
} // namespace rbk

#endif // ~_RBK_CHASIS_INFO_H_
