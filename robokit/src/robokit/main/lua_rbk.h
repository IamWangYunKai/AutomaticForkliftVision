#ifndef _LUA_RBK_H_
#define _LUA_RBK_H_

#include <robokit/config.h>

#define LUA_PLUGIN_MESSAGE_META "lua_plugin_meta"
#define PLUGIN_MESSAGE_META "plugin_meta"
#define SERVICE_MESSAGE_META "service_meta"

#ifdef __cplusplus
RBK_EXTERN "C" {
#endif

    RBK_EXTERN int luaopen_rbk(lua_State* L);

#ifdef __cplusplus
}
#endif

#endif // ~_LUA_RBK_H_
