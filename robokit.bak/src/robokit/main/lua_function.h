#ifndef _LUA_FUNCTION_H_
#define _LUA_FUNCTION_H_

#include <robokit/config.h>

RBK_EXTERN "C" {
#include "lua.h"
    typedef struct {
        const char *name;
        int(*func)(lua_State *);
    }luaDef;
}

RBK_EXTERN luaDef LuaFunc[];

#endif // ~_LUA_FUNCTION_H_