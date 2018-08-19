#ifndef _RBK_MANAGER_H_
#define _RBK_MANAGER_H_

#include <vector>
#include <string>
#include <map>
#include <robokit/core/rbk_core.h>
#include <robokit/core/singleton.h>
#include <robokit/main/lua_rbk.h>

#ifdef RBK_SYS_WINDOWS

RBK_API std::string TCHAR2STRING(TCHAR *STR);

#endif

using namespace rbk;
//class rbk::NPlugin;

typedef struct _deviceInfo {
    std::string m_name;
    std::string m_version;
    bool m_latest;
    std::string m_author;
    std::string m_mail;
    std::string m_desc;
} DeviceInfo;

class NDeviceManager {
public:
    NDeviceManager();
    ~NDeviceManager();
    rbk::core::NPlugin* initDevice(const std::string&);
    rbk::core::NPlugin* addLuaDevice(const std::string&);
    void startDevice(const std::string&);

    bool findDevice(const std::string&, rbk::core::NPlugin**);
    rbk::core::NPlugin* getDevice(const std::string&);

    void bindEvent(const std::string&, const std::string&, const std::string&);
    void unbindEvent(const std::string&, const std::string&, const std::string&);
    void runScript();
    void addPluginPath(const std::string& path);
    std::vector<std::string> getPluginPaths();
    //void startRPCServer(int);
    //void connectRPCServer(std::string, int);
    //std::string sendRPCRequest(std::string);
    std::vector<DeviceInfo> getDeviceInfos();
    bool getDeviceInfo(const std::string&, DeviceInfo&);
    std::vector<NPluginInterfaceProvider*> m_plugin_providers;
private:
    std::string renameDevice(const std::string&, int&);
    std::vector<rbk::core::NPlugin*> m_devices;
    std::map<std::string, int> m_device_count_map;
    std::map<std::string, std::string> m_event_func_map;
    rbk::core::RBK m_RBK;
    std::vector<DeviceInfo> m_device_info;
    //ViewPlugin m_view_plugin;
};

typedef rbk::core::NormalSingleton<NDeviceManager> DeviceManager;
#endif //~_RBK_MANAGER_H_