#ifndef _RBK_CORE_SERVICE_H_
#define _RBK_CORE_SERVICE_H_

#include <robokit/core/lua_wrapper.h>

RBK_EXTERN "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

// overload operator<< for std::vector
namespace std {
    template <typename T>
    std::ostream& operator<< (std::ostream& os, const typename std::vector<T>& v) {
        os << std::string("[");
        for (typename std::vector<T>::const_iterator i = v.begin(); i != v.end(); ++i) {
            if (i != (v.end() - 1)) {
                os << *i << std::string(",");
            }
            else {
                os << *i;
            }
        }
        os << std::string("]");
        return os;
    }
}

namespace rbk {
    namespace core {
        //------------------------------param_traits--------------------------------//
        template <typename _Tp>
        struct param_traits {
            static _Tp get_param(lua_State *L, const int& index) {
                return static_cast<_Tp>(lua_tonumber(L, index));
            }
        };

        // specialization for std::vector<_Tp>
        template <typename _Tp>
        struct param_traits<std::vector<_Tp>> {
            static std::vector<_Tp> get_param(lua_State *L, const int& index) {
                std::vector<_Tp> p;
                rbk::lua::getLuaStackTraits(L, p, index);
                return p;
            }
        };

        template <>
        struct param_traits<bool> {
            static bool get_param(lua_State *L, const int& index) {
                return lua_toboolean(L, index) != 0;
            }
        };

        template <>
        struct param_traits<double> {
            static double get_param(lua_State *L, const int& index) {
                return static_cast<double>(lua_tonumber(L, index));
            }
        };

        template <>
        struct param_traits<float> {
            static float get_param(lua_State *L, const int& index) {
                return static_cast<float>(lua_tonumber(L, index));
            }
        };

        template <>
        struct param_traits<int> {
            static int get_param(lua_State *L, const int& index) {
                return static_cast<int>(lua_tointeger(L, index));
            }
        };

        template <>
        struct param_traits<unsigned int> {
            static unsigned int get_param(lua_State *L, const int& index) {
                return static_cast<unsigned int>(lua_tointeger(L, index));
            }
        };

        template <>
        struct param_traits<int16_t> {
            static int16_t get_param(lua_State *L, const int& index) {
                return static_cast<int16_t>(lua_tointeger(L, index));
            }
        };

        template <>
        struct param_traits<uint16_t> {
            static uint16_t get_param(lua_State *L, const int& index) {
                return static_cast<uint16_t>(lua_tointeger(L, index));
            }
        };

        template <>
        struct param_traits<int64_t> {
            static int64_t get_param(lua_State *L, const int& index) {
                return static_cast<int64_t>(lua_tonumber(L, index));
            }
        };

        template <>
        struct param_traits<uint64_t> {
            static uint64_t get_param(lua_State *L, const int& index) {
                return static_cast<uint64_t>(lua_tonumber(L, index));
            }
        };

        template <>
        struct param_traits<char> {
            static char get_param(lua_State *L, const int& index) {
                return static_cast<char>(lua_tointeger(L, index));
            }
        };

        template <>
        struct param_traits<unsigned char> {
            static unsigned char get_param(lua_State *L, const int& index) {
                return static_cast<unsigned char>(lua_tointeger(L, index));
            }
        };

        template <>
        struct param_traits<std::string> {
            static std::string get_param(lua_State *L, const int& index) {
                size_t len;
                const char* str = lua_tolstring(L, index, &len);
                if (str == NULL) {
                    return std::string("");
                }
                else {
                    return std::string(str, len);
                }
            }
        };

        template <>
        struct param_traits<const char*> {
            static const char* get_param(lua_State *L, const int& index) {
                size_t len;
                // BUG: str 指向的内存可能被lua的垃圾回收器回收
                const char* str = lua_tolstring(L, index, &len);
                if (str == NULL) {
                    return "";
                }
                else {
                    return str;
                }
            }
        };

        template <typename _Tp>
        struct param_traits<rbk::lua::Nil<_Tp>> {
            static rbk::lua::Nil<_Tp> get_param(lua_State *L, const int& index) {
                if (lua_isnoneornil(L, index)) {
                    return rbk::lua::Nil<_Tp>();
                }
                else {
                    return rbk::lua::Nil<_Tp>(static_cast<_Tp>(lua_tonumber(L, index)));
                }
            }
        };

        // specialization for rbk::lua::Nil<std::vector<_Tp>>
        template <typename _Tp>
        struct param_traits<rbk::lua::Nil<std::vector<_Tp>>> {
            static rbk::lua::Nil<std::vector<_Tp>> get_param(lua_State *L, const int& index) {
                if (lua_isnoneornil(L, index)) {
                    return rbk::lua::Nil<std::vector<_Tp>>();
                }
                else {
                    std::vector<_Tp> p;
                    rbk::lua::getLuaStackTraits(L, p, index);
                    return rbk::lua::Nil<std::vector<_Tp>>(p);
                }
            }
        };

        template <>
        struct param_traits<rbk::lua::Nil<const char*>> {
            static rbk::lua::Nil<const char*> get_param(lua_State *L, const int& index) {
                if (lua_isnoneornil(L, index)) {
                    return rbk::lua::Nil<const char*>();
                }
                else {
                    size_t len;
                    // BUG: str 指向的内存可能被lua的垃圾回收器回收
                    const char* str = lua_tolstring(L, index, &len);
                    if (str == NULL) {
                        return rbk::lua::Nil<const char*>("");
                    }
                    else {
                        return rbk::lua::Nil<const char*>(str);
                    }
                }
            }
        };

        template <>
        struct param_traits<rbk::lua::Nil<bool>> {
            static rbk::lua::Nil<bool> get_param(lua_State *L, const int& index) {
                if (lua_isnoneornil(L, index)) {
                    return rbk::lua::Nil<bool>();
                }
                else {
                    return rbk::lua::Nil<bool>((lua_toboolean(L, index) != 0));
                }
            }
        };

        template <>
        struct param_traits<rbk::lua::Nil<std::string>> {
            static rbk::lua::Nil<std::string> get_param(lua_State *L, const int& index) {
                if (lua_isnoneornil(L, index)) {
                    return rbk::lua::Nil<std::string>();
                }
                else {
                    size_t len;
                    const char* str = lua_tolstring(L, index, &len);
                    if (str == NULL) {
                        return rbk::lua::Nil<std::string>(std::string(""));
                    }
                    else {
                        return rbk::lua::Nil<std::string>(std::string(str, len));
                    }
                }
            }
        };

        //------------------------------return_traits--------------------------------//
        template <typename _Tp>
        struct return_traits {
            static void set_result(lua_State *L, const lua_Number& r) {
                lua_pushnumber(L, r);
            }
        };

        // specialization for std::vector<_Tp>
        template <typename _Tp>
        struct return_traits<std::vector<_Tp>> {
            static void set_result(lua_State *L, const std::vector<_Tp>& r) {
                rbk::lua::pushLuaStackTraits(L, r);
            }
        };

        // specialization for rbk::lua::Nil<std::vector<_Tp>>
        template <typename _Tp>
        struct return_traits<rbk::lua::Nil<std::vector<_Tp>>> {
            static void set_result(lua_State *L, const rbk::lua::Nil<std::vector<_Tp>>& r) {
                rbk::lua::pushLuaStackTraits(L, r);
            }
        };

        template <>
        struct return_traits<double> {
            static void set_result(lua_State *L, const double& r) {
                lua_pushnumber(L, r);
            }
        };

        template <>
        struct return_traits<float> {
            static void set_result(lua_State *L, const float& r) {
                lua_pushnumber(L, r);
            }
        };

        template <>
        struct return_traits<int> {
            static void set_result(lua_State *L, const int& r) {
                lua_pushinteger(L, r);
            }
        };

        template <>
        struct return_traits<unsigned int> {
            static void set_result(lua_State *L, const unsigned int& r) {
                lua_pushinteger(L, r);
            }
        };

        template <>
        struct return_traits<int16_t> {
            static void set_result(lua_State *L, const int16_t& r) {
                lua_pushinteger(L, r);
            }
        };

        template <>
        struct return_traits<uint16_t> {
            static void set_result(lua_State *L, const uint16_t& r) {
                lua_pushinteger(L, r);
            }
        };

        template <>
        struct return_traits<int64_t> {
            static void set_result(lua_State *L, const int64_t& r) {
                lua_pushnumber(L, static_cast<lua_Number>(r));
            }
        };

        template <>
        struct return_traits<uint64_t> {
            static void set_result(lua_State *L, const uint64_t& r) {
                lua_pushnumber(L, static_cast<lua_Number>(r));
            }
        };

        template <>
        struct return_traits<char> {
            static void set_result(lua_State *L, const char& r) {
                lua_pushinteger(L, r);
            }
        };

        template <>
        struct return_traits<unsigned char> {
            static void set_result(lua_State *L, const unsigned char& r) {
                lua_pushinteger(L, r);
            }
        };

        template <>
        struct return_traits<bool> {
            static void set_result(lua_State *L, const bool& r) {
                lua_pushboolean(L, (r ? 1 : 0));
            }
        };

        template <>
        struct return_traits<const char*> {
            static void set_result(lua_State *L, const char *r) {
                lua_pushstring(L, r);
            }
        };

        template <>
        struct return_traits<std::string> {
            static void set_result(lua_State *L, const std::string& r) {
                lua_pushlstring(L, r.c_str(), r.length());
            }
        };

        //------------------------------member_func--------------------------------//
        template <typename _Tp>
        struct member_func {
            template<typename Cla, typename Func>
            static void execute(lua_State *L, Cla& cla, Func func) {
                _Tp ret = (cla.*func)();
                return_traits<_Tp>::set_result(L, ret);
            }

            template<typename P1, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, Cla& cla, Func func) {
                _Tp ret = (cla.*func)(p1);
                return_traits<_Tp>::set_result(L, ret);
            }

            template<typename P1, typename P2, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, Cla& cla, Func func) {
                _Tp ret = (cla.*func)(p1, p2);
                return_traits<_Tp>::set_result(L, ret);
            }

            template<typename P1, typename P2, typename P3, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, Cla& cla, Func func) {
                _Tp ret = (cla.*func)(p1, p2, p3);
                return_traits<_Tp>::set_result(L, ret);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, Cla& cla, Func func) {
                _Tp ret = (cla.*func)(p1, p2, p3, p4);
                return_traits<_Tp>::set_result(L, ret);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, Cla& cla, Func func) {
                _Tp ret = (cla.*func)(p1, p2, p3, p4, p5);
                return_traits<_Tp>::set_result(L, ret);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6, Cla& cla, Func func) {
                _Tp ret = (cla.*func)(p1, p2, p3, p4, p5, p6);
                return_traits<_Tp>::set_result(L, ret);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6, P7 p7, Cla& cla, Func func) {
                _Tp ret = (cla.*func)(p1, p2, p3, p4, p5, p6, p7);
                return_traits<_Tp>::set_result(L, ret);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6, P7 p7, P8 p8, Cla& cla, Func func) {
                _Tp ret = (cla.*func)(p1, p2, p3, p4, p5, p6, p7, p8);
                return_traits<_Tp>::set_result(L, ret);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6, P7 p7, P8 p8, P9 p9, Cla& cla, Func func) {
                _Tp ret = (cla.*func)(p1, p2, p3, p4, p5, p6, p7, p8, p9);
                return_traits<_Tp>::set_result(L, ret);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6, P7 p7, P8 p8, P9 p9, P10 p10, Cla& cla, Func func) {
                _Tp ret = (cla.*func)(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
                return_traits<_Tp>::set_result(L, ret);
            }
        };

        template <>
        struct member_func<void> {
            template<typename Cla, typename Func>
            static void execute(lua_State *L, Cla& cla, Func func) {
                (cla.*func)();
            }

            template<typename P1, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, Cla& cla, Func func) {
                (cla.*func)(p1);
            }

            template<typename P1, typename P2, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, Cla& cla, Func func) {
                (cla.*func)(p1, p2);
            }

            template<typename P1, typename P2, typename P3, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, Cla& cla, Func func) {
                (cla.*func)(p1, p2, p3);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, Cla& cla, Func func) {
                (cla.*func)(p1, p2, p3, p4);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, Cla& cla, Func func) {
                (cla.*func)(p1, p2, p3, p4, p5);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6, Cla& cla, Func func) {
                (cla.*func)(p1, p2, p3, p4, p5, p6);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6, P7 p7, Cla& cla, Func func) {
                (cla.*func)(p1, p2, p3, p4, p5, p6, p7);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6, P7 p7, P8 p8, Cla& cla, Func func) {
                (cla.*func)(p1, p2, p3, p4, p5, p6, p7, p8);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6, P7 p7, P8 p8, P9 p9, Cla& cla, Func func) {
                (cla.*func)(p1, p2, p3, p4, p5, p6, p7, p8, p9);
            }

            template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename Cla, typename Func>
            static void execute(lua_State *L, P1 p1, P2 p2, P3 p3, P4 p4, P5 p5, P6 p6, P7 p7, P8 p8, P9 p9, P10 p10, Cla& cla, Func func) {
                (cla.*func)(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
            }
        };

        template <typename _Tp>
        struct return_number_traits {
            enum {
                count = 1
            };
        };

        template <>
        struct return_number_traits<void> {
            enum {
                count = 0
            };
        };

        class RBK_API BaseService {
        public:
            BaseService(lua_State* l) {
                m_lua_state = l;
            }
            virtual ~BaseService() {}
            virtual void close() {}
            void setServiceName(std::string s) {
                m_service = s;
            }
            std::string getServiceName() {
                return m_service;
            }
        protected:
            std::string m_service;
            lua_State* m_lua_state;
        };

        template<typename Signature>
        class Service : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis);
                m_initialized = true;
            }

        private:
            bool m_initialized;
            std::function<Signature> m_func;
        };

        template <typename ReturnT, typename Cla, typename Func>
        int callMemFunc(Cla& cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            LogInfo(formatLog("Service", serviceName, cla.name()));
            member_func<ReturnT>::execute(L, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT>
        class Service<ReturnT()> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call() {
                return m_func();
            }

        private:
            bool m_initialized;
            std::function<ReturnT()> m_func;
        };

        template <typename ReturnT, typename P1, typename Cla, typename Func>
        int callMemFunc(Cla &cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            P1 p1 = param_traits<P1>::get_param(L, index);
            if (serviceName == "update_rbk.protocol.Message_MoveTask" || serviceName == "setTaskList") {
                LogInfo(formatLog("Service", serviceName, cla.name(), "too long, ignore"));
            }
            else {
                LogInfo(formatLog("Service", serviceName, cla.name(), p1));
            }
            member_func<ReturnT>::execute(L, p1, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename P1, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT, P1>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT, typename P1>
        class Service<ReturnT(P1)> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis, std::placeholders::_1);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, P1, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call(P1& p1) {
                return m_func(p1);
            }

        private:
            bool m_initialized;
            std::function<ReturnT(P1)> m_func;
        };

        template <typename ReturnT, typename P1, typename P2, typename Cla, typename Func>
        int callMemFunc(Cla &cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            P1 p1 = param_traits<P1>::get_param(L, index);
            P2 p2 = param_traits<P2>::get_param(L, index + 1);
            LogInfo(formatLog("Service", serviceName, cla.name(), p1, p2));
            member_func<ReturnT>::execute(L, p1, p2, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename P1, typename P2, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT, P1, P2>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT, typename P1, typename P2>
        class Service<ReturnT(P1, P2)> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis, std::placeholders::_1, std::placeholders::_2);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, P1, P2, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call(P1& p1, P2& p2) {
                return m_func(p1, p2);
            }

        private:
            bool m_initialized;
            std::function<ReturnT(P1, P2)> m_func;
        };

        template <typename ReturnT, typename P1, typename P2, typename P3, typename Cla, typename Func>
        int callMemFunc(Cla &cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            P1 p1 = param_traits<P1>::get_param(L, index);
            P2 p2 = param_traits<P2>::get_param(L, index + 1);
            P3 p3 = param_traits<P3>::get_param(L, index + 2);
            LogInfo(formatLog("Service", serviceName, cla.name(), p1, p2, p3));
            member_func<ReturnT>::execute(L, p1, p2, p3, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename P1, typename P2, typename P3, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT, P1, P2, P3>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT, typename P1, typename P2, typename P3>
        class Service<ReturnT(P1, P2, P3)> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, P1, P2, P3, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call(P1& p1, P2& p2, P3& p3) {
                return m_func(p1, p2, p3);
            }

        private:
            bool m_initialized;
            std::function<ReturnT(P1, P2, P3)> m_func;
        };

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename Cla, typename Func>
        int callMemFunc(Cla &cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            P1 p1 = param_traits<P1>::get_param(L, index);
            P2 p2 = param_traits<P2>::get_param(L, index + 1);
            P3 p3 = param_traits<P3>::get_param(L, index + 2);
            P4 p4 = param_traits<P4>::get_param(L, index + 3);
            LogInfo(formatLog("Service", serviceName, cla.name(), p1, p2, p3, p4));
            member_func<ReturnT>::execute(L, p1, p2, p3, p4, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT, P1, P2, P3, P4>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT, typename P1, typename P2, typename P3, typename P4>
        class Service<ReturnT(P1, P2, P3, P4)> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, P1, P2, P3, P4, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call(P1& p1, P2& p2, P3& p3, P4& p4) {
                return m_func(p1, p2, p3, p4);
            }

        private:
            bool m_initialized;
            std::function<ReturnT(P1, P2, P3, P4)> m_func;
        };

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename Cla, typename Func>
        int callMemFunc(Cla &cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            P1 p1 = param_traits<P1>::get_param(L, index);
            P2 p2 = param_traits<P2>::get_param(L, index + 1);
            P3 p3 = param_traits<P3>::get_param(L, index + 2);
            P4 p4 = param_traits<P4>::get_param(L, index + 3);
            P5 p5 = param_traits<P5>::get_param(L, index + 4);
            LogInfo(formatLog("Service", serviceName, cla.name(), p1, p2, p3, p4, p5));
            member_func<ReturnT>::execute(L, p1, p2, p3, p4, p5, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT, P1, P2, P3, P4, P5>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5>
        class Service<ReturnT(P1, P2, P3, P4, P5)> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, P1, P2, P3, P4, P5, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call(P1& p1, P2& p2, P3& p3, P4& p4, P5& p5) {
                return m_func(p1, p2, p3, p4, p5);
            }

        private:
            bool m_initialized;
            std::function<ReturnT(P1, P2, P3, P4, P5)> m_func;
        };

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename Cla, typename Func>
        int callMemFunc(Cla &cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            P1 p1 = param_traits<P1>::get_param(L, index);
            P2 p2 = param_traits<P2>::get_param(L, index + 1);
            P3 p3 = param_traits<P3>::get_param(L, index + 2);
            P4 p4 = param_traits<P4>::get_param(L, index + 3);
            P5 p5 = param_traits<P5>::get_param(L, index + 4);
            P6 p6 = param_traits<P6>::get_param(L, index + 5);
            LogInfo(formatLog("Service", serviceName, cla.name(), p1, p2, p3, p4, p5, p6));
            member_func<ReturnT>::execute(L, p1, p2, p3, p4, p5, p6, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT, P1, P2, P3, P4, P5, P6>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
        class Service<ReturnT(P1, P2, P3, P4, P5, P6)> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, P1, P2, P3, P4, P5, P6, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call(P1& p1, P2& p2, P3& p3, P4& p4, P5& p5, P6& p6) {
                return m_func(p1, p2, p3, p4, p5, p6);
            }

        private:
            bool m_initialized;
            std::function<ReturnT(P1, P2, P3, P4, P5, P6)> m_func;
        };

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename Cla, typename Func>
        int callMemFunc(Cla &cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            P1 p1 = param_traits<P1>::get_param(L, index);
            P2 p2 = param_traits<P2>::get_param(L, index + 1);
            P3 p3 = param_traits<P3>::get_param(L, index + 2);
            P4 p4 = param_traits<P4>::get_param(L, index + 3);
            P5 p5 = param_traits<P5>::get_param(L, index + 4);
            P6 p6 = param_traits<P6>::get_param(L, index + 5);
            P7 p7 = param_traits<P7>::get_param(L, index + 6);
            LogInfo(formatLog("Service", serviceName, cla.name(), p1, p2, p3, p4, p5, p6, p7));
            member_func<ReturnT>::execute(L, p1, p2, p3, p4, p5, p6, p7, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT, P1, P2, P3, P4, P5, P6, P7>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
        class Service<ReturnT(P1, P2, P3, P4, P5, P6, P7)> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, P1, P2, P3, P4, P5, P6, P7, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call(P1& p1, P2& p2, P3& p3, P4& p4, P5& p5, P6& p6, P7& p7) {
                return m_func(p1, p2, p3, p4, p5, p6, p7);
            }

        private:
            bool m_initialized;
            std::function<ReturnT(P1, P2, P3, P4, P5, P6, P7)> m_func;
        };

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename Cla, typename Func>
        int callMemFunc(Cla &cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            P1 p1 = param_traits<P1>::get_param(L, index);
            P2 p2 = param_traits<P2>::get_param(L, index + 1);
            P3 p3 = param_traits<P3>::get_param(L, index + 2);
            P4 p4 = param_traits<P4>::get_param(L, index + 3);
            P5 p5 = param_traits<P5>::get_param(L, index + 4);
            P6 p6 = param_traits<P6>::get_param(L, index + 5);
            P7 p7 = param_traits<P7>::get_param(L, index + 6);
            P8 p8 = param_traits<P8>::get_param(L, index + 7);
            LogInfo(formatLog("Service", serviceName, cla.name(), p1, p2, p3, p4, p5, p6, p7, p8));
            member_func<ReturnT>::execute(L, p1, p2, p3, p4, p5, p6, p7, p8, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT, P1, P2, P3, P4, P5, P6, P7, P8>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
        class Service<ReturnT(P1, P2, P3, P4, P5, P6, P7, P8)> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, P1, P2, P3, P4, P5, P6, P7, P8, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call(P1& p1, P2& p2, P3& p3, P4& p4, P5& p5, P6& p6, P7& p7, P8& p8) {
                return m_func(p1, p2, p3, p4, p5, p6, p7, p8);
            }

        private:
            bool m_initialized;
            std::function<ReturnT(P1, P2, P3, P4, P5, P6, P7, P8)> m_func;
        };

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename Cla, typename Func>
        int callMemFunc(Cla &cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            P1 p1 = param_traits<P1>::get_param(L, index);
            P2 p2 = param_traits<P2>::get_param(L, index + 1);
            P3 p3 = param_traits<P3>::get_param(L, index + 2);
            P4 p4 = param_traits<P4>::get_param(L, index + 3);
            P5 p5 = param_traits<P5>::get_param(L, index + 4);
            P6 p6 = param_traits<P6>::get_param(L, index + 5);
            P7 p7 = param_traits<P7>::get_param(L, index + 6);
            P8 p8 = param_traits<P8>::get_param(L, index + 7);
            P9 p9 = param_traits<P9>::get_param(L, index + 8);
            LogInfo(formatLog("Service", serviceName, cla.name(), p1, p2, p3, p4, p5, p6, p7, p8, p9));
            member_func<ReturnT>::execute(L, p1, p2, p3, p4, p5, p6, p7, p8, p9, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT, P1, P2, P3, P4, P5, P6, P7, P8, P9>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9>
        class Service<ReturnT(P1, P2, P3, P4, P5, P6, P7, P8, P9)> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8, std::placeholders::_9);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, P1, P2, P3, P4, P5, P6, P7, P8, P9, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call(P1& p1, P2& p2, P3& p3, P4& p4, P5& p5, P6& p6, P7& p7, P8& p8, P9& p9) {
                return m_func(p1, p2, p3, p4, p5, p6, p7, p8, p9);
            }

        private:
            bool m_initialized;
            std::function<ReturnT(P1, P2, P3, P4, P5, P6, P7, P8, P9)> m_func;
        };

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename Cla, typename Func>
        int callMemFunc(Cla &cla, Func func, lua_State *L, int index, const std::string& serviceName) {
            P1 p1 = param_traits<P1>::get_param(L, index);
            P2 p2 = param_traits<P2>::get_param(L, index + 1);
            P3 p3 = param_traits<P3>::get_param(L, index + 2);
            P4 p4 = param_traits<P4>::get_param(L, index + 3);
            P5 p5 = param_traits<P5>::get_param(L, index + 4);
            P6 p6 = param_traits<P6>::get_param(L, index + 5);
            P7 p7 = param_traits<P7>::get_param(L, index + 6);
            P8 p8 = param_traits<P8>::get_param(L, index + 7);
            P9 p9 = param_traits<P9>::get_param(L, index + 8);
            P10 p10 = param_traits<P10>::get_param(L, index + 9);
            LogInfo(formatLog("Service", serviceName, cla.name(), p1, p2, p3, p4, p5, p6, p7, p8, p9, p10));
            member_func<ReturnT>::execute(L, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, cla, func);
            return return_number_traits<ReturnT>::count;
        }

        template <typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename Cla, typename Func>
        int directCallMemFunc(lua_State *L) {
            intptr_t* buffer = (intptr_t*)lua_touserdata(L, lua_upvalueindex(1));
            std::string serviceName = std::string(lua_tostring(L, lua_upvalueindex(2)));
            return callMemFunc<ReturnT, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10>(*(Cla*)(*buffer), *(Func*)(buffer + sizeof(Cla*) / sizeof(intptr_t)), L, 1, serviceName);
        }

        template<typename ReturnT, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10>
        class Service<ReturnT(P1, P2, P3, P4, P5, P6, P7, P8, P9, P10)> : public BaseService {
        public:
            Service(std::string name, lua_State* l) : BaseService(l) {
                m_initialized = false;
                m_service = name;
            }
            ~Service() {}

            template <typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_func = std::bind(memberFunc, pThis, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8, std::placeholders::_9, std::placeholders::_10);
                m_initialized = true;
                // 向Lua注册
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                intptr_t* buffer = (intptr_t*)lua_newuserdata(m_lua_state, sizeof(pThis) + sizeof(memberFunc));
                memcpy(buffer, &pThis, sizeof(pThis));
                memcpy(buffer + sizeof(pThis) / sizeof(intptr_t), &memberFunc, sizeof(memberFunc));
                lua_pushlstring(m_lua_state, m_service.c_str(), m_service.length());
                lua_pushcclosure(m_lua_state, directCallMemFunc<ReturnT, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, T1, CallBack>, 2);
                lua_settable(m_lua_state, -3);
            }

            ReturnT call(P1& p1, P2& p2, P3& p3, P4& p4, P5& p5, P6& p6, P7& p7, P8& p8, P9& p9, P10& p10) {
                return m_func(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
            }

        private:
            bool m_initialized;
            std::function<ReturnT(P1, P2, P3, P4, P5, P6, P7, P8, P9, P10)> m_func;
        };
    } // namespace core
} // namespace rbk

#endif // ~_RBK_CORE_SERVICE_H_
