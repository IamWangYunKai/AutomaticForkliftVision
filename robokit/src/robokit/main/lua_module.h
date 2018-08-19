#ifndef _LUA_MODULE_H_
#define _LUA_MODULE_H_

#include <robokit/core/singleton.h>
#include <robokit/core/lua_reader.h>
#include <robokit/core/lua_wrapper.h>

#include <boost/program_options/variables_map.hpp>

#include <map>
#include <vector>
using namespace rbk;
struct lua_State;

class NLuaModule {
public:
    NLuaModule();
    virtual ~NLuaModule();
    void		InitLuaGlueFunc();
    bool		RunScript(const char* pFilename);
    bool		RunString(const char* pCommand);
    const char* GetErrorString(void);
    bool		AddFunction(const char* pFunctionName, rbk::lua::LuaFunctionType pFunction);
    //const char *GetStringArgument(int num, const char *pDefault=NULL);
    //double		GetNumberArgument(int num, double dDefault=0.0);
    //bool		GetBoolArgument(int num);
    //void		PushString(const char *pString);
    //void		PushLString(const char* pString, unsigned int size);
    //void		PushNumber(double value);
    //void		PushBool(bool value);
    //void		SetErrorHandler(void(*pErrHandler)(const char *pError)) {m_pErrorHandler = pErrHandler;}

    lua_State* GetLuaState(void) { return m_pScriptContext; }
    // 从Table中读入键所对应的值
    void SetupTable();
    void SetStringArgToTable(const char* key, const char* value);
    const char* GetStringArgFromTable(const char* key);

    void InitRBKConfigTable(const boost::program_options::variables_map& configMap);

private:

    template<typename T>
    void SetRBKConfigTable(const std::string& key, T value) {
        lua_pushlstring(m_pScriptContext, key.c_str(), key.length());
        rbk::lua::pushLuaStackTraits(m_pScriptContext, value);
        lua_settable(m_pScriptContext, -3);
    }

    template<typename T>
    void SetRBKConfigTable(const std::string& key, std::vector<T> value) {
        lua_pushlstring(m_pScriptContext, key.c_str(), key.length());
        rbk::lua::pushLuaStackTraits(m_pScriptContext, value);
        lua_settable(m_pScriptContext, -3);
    }

    template<typename T>
    void SetRBKConfigTable(const std::string& key, rbk::lua::Nil<T> value) {
        lua_pushlstring(m_pScriptContext, key.c_str(), key.length());
        rbk::lua::pushLuaStackTraits(m_pScriptContext, value);
        lua_settable(m_pScriptContext, -3);
    }

    lua_State* m_pScriptContext;
};

typedef rbk::core::MeyersSingleton<NLuaModule> LuaModule;

#endif // ~_LUA_MODULE_H_
