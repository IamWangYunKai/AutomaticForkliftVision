#ifndef _RBK_INTERFACE_H_
#define _RBK_INTERFACE_H_

#include <robokit/core/rbk_core.h>
#include <robokit/core/plugin.h>

namespace rbk {
    class RBK_API NPluginInterface : public rbk::core::NPlugin {
    public:
        NPluginInterface() {}
    private:
        NPluginInterface(const NPluginInterface&);
        NPluginInterface& operator=(const NPluginInterface&);
    };

    class RBK_API NPluginInterfaceProvider : public rbk::core::Provider {
    public:
        static const unsigned int RBK_INTERFACE_MAJOR_VERSION;
        static const unsigned int RBK_INTERFACE_MINOR_VERSION;
        static const std::string RBK_PROVIDER_TYPE;
        unsigned int getMajorVersion() const { return RBK_INTERFACE_MAJOR_VERSION; }
        unsigned int getMinorVersion() const { return RBK_INTERFACE_MINOR_VERSION; }
        unsigned int getSelfMajorVersion() const { return SELF_RBK_INTERFACE_MAJOR_VERSION; }
        unsigned int getSelfMinorVersion() const { return SELF_RBK_INTERFACE_MINOR_VERSION; }
        std::string getName() const { return RBK_INTERFACE_NAME; }
        std::string getCompleteVersion() const { return COMPLETE_VERSION; }
        virtual NPluginInterface* create() const = 0;
    protected:
        unsigned int SELF_RBK_INTERFACE_MAJOR_VERSION;
        unsigned int SELF_RBK_INTERFACE_MINOR_VERSION;
        std::string RBK_INTERFACE_NAME;
        std::string COMPLETE_VERSION;
    private:
        std::string rbkGetType() const { return RBK_PROVIDER_TYPE; }
    };
}

#endif // ~_RBK_INTERFACE_H_
