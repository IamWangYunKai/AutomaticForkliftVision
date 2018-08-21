#ifndef _RBK_CHASIS_MODEL_H_
#define _RBK_CHASIS_MODEL_H_

#include <robokit/config.h>
#include <robokit/utils/json.h>
#include <robokit/core/mutex.h>
#include <robokit/chasis/calibration.h>

#include <boost/signals2.hpp>

namespace rbk {
    namespace chasis {

        class RBK_API Model {
        private:
            struct object_creator {
                // This constructor does nothing more than ensure that Instance()
                // is called before main() begins, thus creating the static
                // SingletonClass object before multithreading race issues can come up.
                object_creator() { Model::Instance(); }
                inline void do_nothing() const { }
            };

            static object_creator create_object;

            Model();

            ~Model();

            Model(const Model&);

            Model& operator=(const Model&);

        public:

            static Model* Instance();

            bool set(const rbk::utils::json& m, const std::string& md5, const std::vector<char>& rawJson, std::string& errMsg  = std::string(""));

            rbk::utils::json getJson();

            std::vector<char> getRawJson();

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

            bool verify(const rbk::utils::json& m, std::string& errMsg  = std::string(""));

            std::string name();

            std::string md5();

            void print();

            boost::signals2::connection connectChangedSignal(const boost::signals2::signal<void()>::slot_type& func);

        private:

            rbk::utils::json _jsonDoc;

            std::vector<char> _rawJson;

            std::string _name;

            std::string _md5;

            rbk::rwMutex _mutex;

            boost::signals2::signal<void()> _changed;
        };
    } // namespace chasis
} // namespace rbk

#endif // ~_RBK_CHASIS_MODEL_H_
