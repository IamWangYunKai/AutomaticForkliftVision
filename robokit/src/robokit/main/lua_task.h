#ifndef _LUA_TASK_H_
#define _LUA_TASK_H_

#include <string>
#include <map>
#include <vector>
#include <robokit/core/event.h>

RBK_EXTERN "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

struct _devEventMsg {
    std::string dev_name;
    std::string event_name;
    std::string func_name;
};

class taskMsg {
public:
    taskMsg() : m_running(false), m_init(false) {}
    ~taskMsg() {}
    std::string m_task_name;
    std::vector<_devEventMsg> m_event_list;
    rbk::core::Event* m_finish_event;
    bool m_running;
    bool m_init;
};

#define TASK_MESSAGE_META "task_meta"

RBK_EXTERN "C" int luaopen_task(lua_State* L);

#endif // ~_LUA_TASK_H_
