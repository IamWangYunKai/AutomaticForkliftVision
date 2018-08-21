#ifndef _RBK_CORE_PROVIDER_H_
#define _RBK_CORE_PROVIDER_H_

#include <robokit/config.h>
#include <string>

namespace rbk {
    namespace core {
        class Host;

        class RBK_API Provider {
            friend class Host;

        public:
            virtual ~Provider();
            virtual unsigned int getMajorVersion() const = 0;
            virtual unsigned int getMinorVersion() const = 0;
            virtual unsigned int getSelfMajorVersion() const = 0;
            virtual unsigned int getSelfMinorVersion() const = 0;
            virtual std::string getName() const = 0;
            bool isCompatible(const Host& host) const;
        private:
            virtual std::string rbkGetType() const = 0;
        };
    } // namespace core
}   // namespace rbk

#endif // ~_RBK_CORE_PROVIDER_H_
