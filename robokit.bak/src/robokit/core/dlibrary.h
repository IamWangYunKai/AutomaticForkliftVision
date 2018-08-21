#ifndef _RBK_CORE_DLIBRARY_H_
#define _RBK_CORE_DLIBRARY_H_

#include <robokit/config.h>

#include <string>

#ifdef RBK_SYS_WINDOWS
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

namespace rbk {
    namespace core {
        class RBK_API DLibrary {
        public:
            static DLibrary* load(const std::string& path);
            ~DLibrary();
            void* getSymbol(const std::string& symbol);
        private:
            DLibrary();
            DLibrary(void* handle);

        private:
            void* handle; ///< Library handle.
        };
    } // namespace core
}   // namespace rbk

#endif // ~_RBK_CORE_DLIBRARY_H_
