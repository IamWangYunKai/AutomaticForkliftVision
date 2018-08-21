#ifndef _RBK_CORE_LOADER_H_
#define _RBK_CORE_LOADER_H_

#include <robokit/config.h>
#include <robokit/core/host.h>

#include <string>
#include <map>

namespace rbk {
    namespace core {
        class DLibrary;

        // 负责将路径下所有的插件都载入
        class RBK_API Loader {
        public:
            ~Loader();
            bool load(const std::string& pluginName);
            bool load(const std::string& folder, const std::string& pluginName);
            int loadFromFolder(const std::string& folder, const bool recursive = false);
            bool unload(const std::string& pluginName);
            void unloadAll();
            bool addProvider(Provider* provider);
            void getLoadedPlugins(std::vector<const std::string*>& pluginNames) const;
            bool isLoaded(const std::string& pluginName) const;
            void addLoaderPath(const std::string& path);
            std::vector<std::string> getLoaderPaths() const;
        protected:
            Loader();
            void registerType(const std::string& type, const unsigned int majorVersion, const unsigned int minorVersion);
            const std::list<Provider*>* getProviders(const std::string& type) const;
        private:
            static std::string getPluginName(const std::string& path);
            static std::string resolvePathExtension(const std::string& path);
        private:
            /// Signature for the plugin's registration function
            typedef bool fnRegisterPlugin(Host&);
            typedef std::map<std::string, DLibrary*> LibMap;

            LibMap libraries;   ///< Map containing the loaded libraries
            Host host;          ///< Host app proxy, holding all providers
            std::vector<std::string> loaderPaths;
        };
    } // namespace core
}   // namespace rbk

#endif // ~_RBK_CORE_LOADER_H_
