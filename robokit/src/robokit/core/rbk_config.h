#ifndef _RBK_CONFIG_H_
#define _RBK_CONFIG_H_

#include <robokit/config.h>

#include <boost/program_options/variables_map.hpp>

namespace rbk {
    class RBK_API Config {
    private:
        struct object_creator {
            // This constructor does nothing more than ensure that Instance()
            // is called before main() begins, thus creating the static
            // SingletonClass object before multithreading race issues can come up.
            object_creator() { Config::Instance(); }
            inline void do_nothing() const { }
        };

        static object_creator create_object;

        Config();

        ~Config();

        Config(const Config&);

        Config& operator=(const Config&);

    public:

        static Config* Instance();

        void init(const boost::program_options::variables_map& vm);

        template<typename T>
        bool get(const std::string& key, T& value) const {
            if (!_init.load()) { return false; }
            if (_configMap.count(key)) {
                try {
                    value = _configMap[key].as<T>();
                    return true;
                }
                catch (boost::bad_any_cast&) {
                    return false;
                }
            }
            else {
                return false;
            }
        }

        const boost::program_options::variables_map& getConfigMap() const;

    private:
        std::atomic<bool> _init;
        boost::program_options::variables_map _configMap;
    };
} // namespace rbk

#endif // ~_RBK_CONFIG_H_
