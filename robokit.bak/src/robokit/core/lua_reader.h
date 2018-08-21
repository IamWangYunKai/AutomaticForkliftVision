#ifndef _RBK_LUA_READER_H_
#define _RBK_LUA_READER_H_

#include <robokit/config.h>
#include <string>
#include <robokit/core/lua_wrapper.h>

struct lua_State;

namespace rbk {
    namespace lua {
        RBK_EXTERN "C" {
            typedef int(*LuaFunctionType)(struct lua_State *pLuaState);
        };

        class RBK_API LuaReader {
        public:
            LuaReader();
            ~LuaReader();
            bool LoadFile(const char* str);
            template<typename T>
            bool LoadFromLua(const char* str, T&) {
                return false;
            }

            void    SetLuaState(lua_State*);
            bool    AddFunction(const char *pFunctionName, LuaFunctionType pFunction);

            void runLuaFunc(const char* func_name) {
                executeLuaFunc(m_lua_state, func_name);
            }

            template<typename TIN1>
            void runLuaFunc(const TIN1& in1, const char* func_name) {
                executeLuaFunc(in1, m_lua_state, func_name);
            }

            template<typename TOUT1>
            void runLuaFunc(const char* func_name, TOUT1& out1) {
                executeLuaFunc(m_lua_state, func_name, out1);
            }

            template<typename TIN1, typename TOUT1, typename TOUT2>
            void runLuaFunc(const TIN1& in1, const char* func_name, TOUT1& out1, TOUT2& out2) {
                executeLuaFunc(in1, m_lua_state, func_name, out1, out2);
            }

            template<typename TIN1, typename TOUT1, typename TOUT2, typename TOUT3>
            void runLuaFunc(const TIN1& in1, const char* func_name, TOUT1& out1, TOUT2& out2, TOUT3& out3) {
                executeLuaFunc(in1, m_lua_state, func_name, out1, out2, out3);
            }

            template<typename TIN1, typename TIN2>
            void runLuaFunc(const TIN1& in1, const TIN2& in2, const char* func_name) {
                executeLuaFunc(in1, in2, m_lua_state, func_name);
            }

            template<typename TIN1, typename TIN2, typename TOUT1>
            void runLuaFunc(const TIN1& in1, const TIN2& in2, const char* func_name, TOUT1& out1) {
                executeLuaFunc(in1, in2, m_lua_state, func_name, out1);
            }

            template<typename TIN1, typename TOUT1>
            void runLuaFunc(const TIN1& in1, const char* func_name, TOUT1& out1) {
                executeLuaFunc(in1, m_lua_state, func_name, out1);
            }
            /*
            template<>
            bool LoadFromLua<std::string>(const char* str, std::string& res);

            template<>
            bool LoadFromLua<int>(const char* str, int& res);

            template<>
            bool LoadFromLua<double>(const char* str, double& res) ;

            template<>
            bool LoadFromLua<bool>(const char* str, bool& res);*/
        private:
            lua_State	*m_lua_state;
        };
        template<>
        RBK_API bool LuaReader::LoadFromLua<std::string>(const char* str, std::string& res);

        template<>
        RBK_API bool LuaReader::LoadFromLua<int>(const char* str, int& res);

        template<>
        RBK_API bool LuaReader::LoadFromLua<double>(const char* str, double& res);

        template<>
        RBK_API bool LuaReader::LoadFromLua<bool>(const char* str, bool& res);
    } // namespace lua
} // namespace rbk

#endif // ~_RBK_LUA_READER_H_
