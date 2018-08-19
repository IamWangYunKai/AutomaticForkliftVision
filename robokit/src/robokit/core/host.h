#ifndef _RBK_CORE_HOST_H_
#define _RBK_CORE_HOST_H_

#include <robokit/config.h>
#include <robokit/core/provider.h>

#include <list>
#include <map>
#include <vector>

namespace rbk {
    namespace core {
        class RBK_API Host {
            friend class Loader;
            friend class Provider;

        public:
            bool add(Provider* provider);

        private:
            Host();
            ~Host();
            bool knows(const std::string& type) const;
            unsigned int getMajorVersion(const std::string& type) const;
            unsigned int getMinorVersion(const std::string& type) const;
            void registerType(const std::string& type, const unsigned int majorVersion, const unsigned int minorVersion);
            const std::list<Provider*>* getProviders(const std::string& type) const;
            void clearProviders();
            bool validateProvider(Provider* provider) const;
            bool registerProvider(Provider* provider);
            void cancelAddictions();
            bool confirmAddictions();

        private:
            struct ProviderInfo {
                unsigned int majorVersion;
                unsigned int minorVersion;
                std::list<Provider*> providers;
            };

            typedef std::map<std::string, ProviderInfo> ProvidersMap;
            typedef std::map<std::string, std::list<Provider*>> TempProvidersMap;

            ProvidersMap knownTypes;        ///< Map of registered types.
            TempProvidersMap addRequests;   ///< Temporarily added providers
        };
    } // namespace core
}   // namespace rbk

#endif // ~_RBK_CORE_HOST_H_
