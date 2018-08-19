#ifndef _RBK_CORE_EVENT_H_
#define _RBK_CORE_EVENT_H_

#include <string>
#include <vector>

#include <robokit/config.h>
#include <robokit/core/thread_pool.h>
#include <robokit/core/lua_wrapper.h>
#include <robokit/core/logger.h>
#include <robokit/core/mutex.h>
#include <robokit/core/error.h>

namespace rbk {
    namespace core {
        class RBK_API Event {
        public:
            Event(const std::string& dev_name, const std::string& event_name, lua_State* l);
            ~Event();
            void bind(const std::string& func_name);
            void unbind(const std::string& func_name);
        private:
            Event(const Event&);
            Event& operator=(const Event&);

            std::string m_bind_func;
            std::string m_device_name;
            std::string m_event_name;
            lua_State* m_lua_state;
            rbk::mutex m_bind_mutex;
            rbk::mutex m_lua_mutex;
        public:

            template<typename R>
            R fire() {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return R();
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func]() -> void {
                    this->doEventFunc(bind_func);
                });
                return R();
            }

            template<typename P0>
            void fire(const P0& p0) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0]() -> void {
                    this->doEventFunc<P0>(bind_func, p0);
                });
            }

            template<typename P0, typename P1>
            void fire(const P0& p0, const  P1& p1) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1]() -> void {
                    this->doEventFunc<P0, P1>(bind_func, p0, p1);
                });
            }

            template<typename P0, typename P1, typename P2>
            void fire(const P0& p0, const P1& p1, const P2& p2) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2]() -> void {
                    this->doEventFunc<P0, P1, P2>(bind_func, p0, p1, p2);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3]() -> void {
                    this->doEventFunc<P0, P1, P2, P3>(bind_func, p0, p1, p2, p3);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4>(bind_func, p0, p1, p2, p3, p4);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4, p5));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4, p5]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4, P5>(bind_func, p0, p1, p2, p3, p4, p5);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4, p5, p6));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4, p5, p6]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4, P5, P6>(bind_func, p0, p1, p2, p3, p4, p5, p6);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4, p5, p6, p7));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4, p5, p6, p7]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4, P5, P6, P7>(bind_func, p0, p1, p2, p3, p4, p5, p6, p7);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4, P5, P6, P7, P8>(bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9>(bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10>(bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11>(bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12>(bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13>(bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13);
                });
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13, typename P14>
            void fire(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13, const P14& p14) {
                rbk::lockGuard l(m_bind_mutex);
                if (m_bind_func == "") {
                    //LogWarn("fireEvent: " << m_event_name << " @ " << m_device_name << " has not been bound");
                    return;
                }
                LogInfo(formatLog("Event", m_event_name, m_device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14));
                std::string bind_func = m_bind_func;
                rbk::ThreadPool::Instance()->schedule([this, bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14]() -> void {
                    this->doEventFunc<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14>(bind_func, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14);
                });
            }

            void doEventFunc(const std::string& func_name) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                int ret = lua_resume(m_lua_state, 0);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0>
            void doEventFunc(const std::string& func_name, const P0& p0) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                int ret = lua_resume(m_lua_state, 1);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                int ret = lua_resume(m_lua_state, 2);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                int ret = lua_resume(m_lua_state, 3);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                int ret = lua_resume(m_lua_state, 4);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                int ret = lua_resume(m_lua_state, 5);
                if (ret == 0) {
                    lua_settop(m_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                rbk::lua::pushLuaStackTraits(m_lua_state, p5);
                int ret = lua_resume(m_lua_state, 6);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                rbk::lua::pushLuaStackTraits(m_lua_state, p5);
                rbk::lua::pushLuaStackTraits(m_lua_state, p6);
                int ret = lua_resume(m_lua_state, 7);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                rbk::lua::pushLuaStackTraits(m_lua_state, p5);
                rbk::lua::pushLuaStackTraits(m_lua_state, p6);
                rbk::lua::pushLuaStackTraits(m_lua_state, p7);
                int ret = lua_resume(m_lua_state, 8);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                rbk::lua::pushLuaStackTraits(m_lua_state, p5);
                rbk::lua::pushLuaStackTraits(m_lua_state, p6);
                rbk::lua::pushLuaStackTraits(m_lua_state, p7);
                rbk::lua::pushLuaStackTraits(m_lua_state, p8);
                int ret = lua_resume(m_lua_state, 9);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                rbk::lua::pushLuaStackTraits(m_lua_state, p5);
                rbk::lua::pushLuaStackTraits(m_lua_state, p6);
                rbk::lua::pushLuaStackTraits(m_lua_state, p7);
                rbk::lua::pushLuaStackTraits(m_lua_state, p8);
                rbk::lua::pushLuaStackTraits(m_lua_state, p9);
                int ret = lua_resume(m_lua_state, 10);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                rbk::lua::pushLuaStackTraits(m_lua_state, p5);
                rbk::lua::pushLuaStackTraits(m_lua_state, p6);
                rbk::lua::pushLuaStackTraits(m_lua_state, p7);
                rbk::lua::pushLuaStackTraits(m_lua_state, p8);
                rbk::lua::pushLuaStackTraits(m_lua_state, p9);
                rbk::lua::pushLuaStackTraits(m_lua_state, p10);
                int ret = lua_resume(m_lua_state, 11);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                rbk::lua::pushLuaStackTraits(m_lua_state, p5);
                rbk::lua::pushLuaStackTraits(m_lua_state, p6);
                rbk::lua::pushLuaStackTraits(m_lua_state, p7);
                rbk::lua::pushLuaStackTraits(m_lua_state, p8);
                rbk::lua::pushLuaStackTraits(m_lua_state, p9);
                rbk::lua::pushLuaStackTraits(m_lua_state, p10);
                rbk::lua::pushLuaStackTraits(m_lua_state, p11);
                int ret = lua_resume(m_lua_state, 12);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                rbk::lua::pushLuaStackTraits(m_lua_state, p5);
                rbk::lua::pushLuaStackTraits(m_lua_state, p6);
                rbk::lua::pushLuaStackTraits(m_lua_state, p7);
                rbk::lua::pushLuaStackTraits(m_lua_state, p8);
                rbk::lua::pushLuaStackTraits(m_lua_state, p9);
                rbk::lua::pushLuaStackTraits(m_lua_state, p10);
                rbk::lua::pushLuaStackTraits(m_lua_state, p11);
                rbk::lua::pushLuaStackTraits(m_lua_state, p12);
                int ret = lua_resume(m_lua_state, 13);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                rbk::lua::pushLuaStackTraits(m_lua_state, p5);
                rbk::lua::pushLuaStackTraits(m_lua_state, p6);
                rbk::lua::pushLuaStackTraits(m_lua_state, p7);
                rbk::lua::pushLuaStackTraits(m_lua_state, p8);
                rbk::lua::pushLuaStackTraits(m_lua_state, p9);
                rbk::lua::pushLuaStackTraits(m_lua_state, p10);
                rbk::lua::pushLuaStackTraits(m_lua_state, p11);
                rbk::lua::pushLuaStackTraits(m_lua_state, p12);
                rbk::lua::pushLuaStackTraits(m_lua_state, p13);
                int ret = lua_resume(m_lua_state, 14);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13, typename P14>
            void doEventFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13, const P14& p14) {
                rbk::lockGuard l(m_lua_mutex);
                lua_getglobal(m_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(m_lua_state, p0);
                rbk::lua::pushLuaStackTraits(m_lua_state, p1);
                rbk::lua::pushLuaStackTraits(m_lua_state, p2);
                rbk::lua::pushLuaStackTraits(m_lua_state, p3);
                rbk::lua::pushLuaStackTraits(m_lua_state, p4);
                rbk::lua::pushLuaStackTraits(m_lua_state, p5);
                rbk::lua::pushLuaStackTraits(m_lua_state, p6);
                rbk::lua::pushLuaStackTraits(m_lua_state, p7);
                rbk::lua::pushLuaStackTraits(m_lua_state, p8);
                rbk::lua::pushLuaStackTraits(m_lua_state, p9);
                rbk::lua::pushLuaStackTraits(m_lua_state, p10);
                rbk::lua::pushLuaStackTraits(m_lua_state, p11);
                rbk::lua::pushLuaStackTraits(m_lua_state, p12);
                rbk::lua::pushLuaStackTraits(m_lua_state, p13);
                rbk::lua::pushLuaStackTraits(m_lua_state, p14);
                int ret = lua_resume(m_lua_state, 15);
                if (ret != 0) {
                    LogFatal(formatLog("Event", m_event_name, m_device_name, std::string("Run event function error: ").append(lua_tostring(m_lua_state, lua_gettop(m_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    // TODO 错误处理，应该throw出来
                }
                lua_settop(m_lua_state, 0);
            }
        };
    } // namespace core
} // namespace rbk

#endif // ~_RBK_CORE_EVENT_H_
