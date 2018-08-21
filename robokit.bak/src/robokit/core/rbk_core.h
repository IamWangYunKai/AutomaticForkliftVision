#ifndef _RBK_CORE_H_
#define _RBK_CORE_H_

#include <robokit/config.h>

#include <robokit/core/loader.h>
#include <robokit/core/interface.h>
#include <robokit/core/provider.h>
#include <robokit/core/singleton.h>

////////////////////////////////////////////////////////////
// Macro that generate the provider declaration
////////////////////////////////////////////////////////////
#define RBK_PROVIDER_SOURCE(TYPE, MajorVersion, MinorVersion)                           \
    const std::string TYPE##Provider::RBK_PROVIDER_TYPE = #TYPE;                        \
    const unsigned int TYPE##Provider::RBK_INTERFACE_MAJOR_VERSION = MajorVersion;      \
    const unsigned int TYPE##Provider::RBK_INTERFACE_MINOR_VERSION = MinorVersion;

////////////////////////////////////////////////////////////
// Macro that helps plugins generating their provider implementations
// PRE: SPECIALIZED_TYPE must inherit from BASE_TYPE
////////////////////////////////////////////////////////////
#define RBK_INHERIT_PROVIDER(SPECIALIZED_TYPE, BASE_TYPE, Version)                      \
    typedef rbk::core::NormalSingleton<SPECIALIZED_TYPE> GRBKHandle;                    \
    class SPECIALIZED_TYPE##Provider: public BASE_TYPE##Provider {                      \
    public:                                                                             \
        SPECIALIZED_TYPE##Provider() {                                                  \
            RBK_INTERFACE_NAME = #SPECIALIZED_TYPE;                                     \
            SELF_RBK_INTERFACE_MAJOR_VERSION = ROBOKIT_VERSION_MAJOR;                   \
            SELF_RBK_INTERFACE_MINOR_VERSION = ROBOKIT_VERSION_MINOR;                   \
            COMPLETE_VERSION = Version;                                                 \
        }                                                                               \
        BASE_TYPE* create() const{ return GRBKHandle::Instance(); }                     \
    };

#define RBKHandle GRBKHandle::Instance()

#define RBK_INHERIT_SOURCE(SPECIALIZED_TYPE)                                   \
    RBK_EXTERN "C" RBK_PLUGIN_EXPORT bool plugin_connect(rbk::core::Host& host) {  \
        return host.add(new SPECIALIZED_TYPE##Provider());                     \
    }

namespace rbk {
    namespace core {
        class RBK_API RBK : public Loader {
        public:
            RBK() {}

            template<typename ProviderType>
            void acceptProviderType() {
                Loader::registerType(
                    ProviderType::RBK_PROVIDER_TYPE,
                    ProviderType::RBK_INTERFACE_MAJOR_VERSION,
                    ProviderType::RBK_INTERFACE_MINOR_VERSION
                );
            }

            template<typename ProviderType>
            void getProviders(std::vector<ProviderType*>& providers) {
                const std::list<Provider*>* lst = Loader::getProviders(ProviderType::RBK_PROVIDER_TYPE);
                if (!lst) return;
                providers.reserve(providers.size() + lst->size());
                std::list<Provider*>::const_iterator it;
                for (it = lst->begin(); it != lst->end(); ++it)
                    providers.push_back(static_cast<ProviderType*>(*it));
            }

            template<typename ProviderType>
            void loadProvider(const std::string& pluginName, ProviderType*& provider) {
#ifdef RBK_SYS_WINDOWS
                load(pluginName);
#else
                std::string libName = "lib";
                libName.append(pluginName);
                load(libName);
#endif
                std::list<Provider*>::const_iterator it;
                const std::list<Provider*>* lst = Loader::getProviders(ProviderType::RBK_PROVIDER_TYPE);
                for (it = lst->begin(); it != lst->end(); ++it) {
                    provider = dynamic_cast<ProviderType*>(*it);
                    if (provider && provider->getName() == pluginName) {
                        return;
                    }
                    else {
                        provider = nullptr;
                    }
                }
                
            }
        };
    } // namespace core
} // namespace rbk

#endif // ~_RBK_CORE_H_
