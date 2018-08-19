#ifndef _RBK_CORE_DELEGATE_H_
#define _RBK_CORE_DELEGATE_H_

#include <string>
#include <vector>

#include <robokit/config.h>
#include <robokit/core/thread_pool.h>
#include <robokit/core/lua_wrapper.h>
#include <robokit/core/logger.h>
#include <robokit/core/mutex.h>
#include <robokit/core/error.h>

#include <boost/tuple/tuple.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/adapted/boost_tuple.hpp>

namespace rbk {
    namespace core {
        struct popLuaStateWrapper {
            template<typename T>
            void operator()(lua_State* L, T& t) const {
                // when there is no return value
                if (lua_gettop(L) == 0) {
                    return;
                }
                rbk::lua::popFirstLuaStackTraits(L, t);
            }

            template<typename T>
            void operator()(lua_State* L, std::vector<T>& t) const {
                t.clear();
                // when there is no return value
                if (lua_gettop(L) == 0) {
                    return;
                }
                rbk::lua::popFirstLuaStackTraits(L, t);
            }
        };

        class RBK_API Delegate {
        public:
            Delegate(const std::string& device_name, const std::string& delegate_name, lua_State* l);
            ~Delegate();
            void bind(const std::string& func_name);
            void unbind(const std::string& func_name);
        private:
            Delegate(const Delegate&);
            Delegate& operator=(const Delegate&);

            std::string _delegate_func_name;
            std::string _device_name;
            std::string _delegate_name;
            lua_State* _lua_state;
            rbk::mutex _bind_mutex;
            rbk::mutex _lua_mutex;
        public:
            bool delegateFuncExists();

            template<typename R>
            R call() {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name));
                return doDelegateFunc<R>(func_name);
            }

            template<typename P0>
            void call(const P0& p0) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0));
                doDelegateFunc(func_name, p0);
            }

            template<typename R, typename P0>
            R call(const P0& p0) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0));
                return doDelegateFunc<R, P0>(func_name, p0);
            }

            template<typename P0, typename P1>
            void call(const P0& p0, const P1& p1) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1));
                doDelegateFunc(func_name, p0, p1);
            }

            template<typename R, typename P0, typename P1>
            R call(const P0& p0, const P1& p1) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1));
                return doDelegateFunc<R, P0, P1>(func_name, p0, p1);
            }

            template<typename P0, typename P1, typename P2>
            void call(const P0& p0, const P1& p1, const P2& p2) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2));
                doDelegateFunc(func_name, p0, p1, p2);
            }

            template<typename R, typename P0, typename P1, typename P2>
            R call(const P0& p0, const P1& p1, const P2& p2) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2));
                return doDelegateFunc<R, P0, P1, P2>(func_name, p0, p1, p2);
            }

            template<typename P0, typename P1, typename P2, typename P3>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3));
                doDelegateFunc(func_name, p0, p1, p2, p3);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3));
                return doDelegateFunc<R, P0, P1, P2, P3>(func_name, p0, p1, p2, p3);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4));
                return doDelegateFunc<R, P0, P1, P2, P3, P4>(func_name, p0, p1, p2, p3, p4);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4, p5);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5));
                return doDelegateFunc<R, P0, P1, P2, P3, P4, P5>(func_name, p0, p1, p2, p3, p4, p5);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4, p5, p6);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6));
                return doDelegateFunc<R, P0, P1, P2, P3, P4, P5, P6>(func_name, p0, p1, p2, p3, p4, p5, p6);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4, p5, p6, p7);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7));
                return doDelegateFunc<R, P0, P1, P2, P3, P4, P5, P6, P7>(func_name, p0, p1, p2, p3, p4, p5, p6, p7);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8));
                return doDelegateFunc<R, P0, P1, P2, P3, P4, P5, P6, P7, P8>(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9));
                return doDelegateFunc<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9>(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10));
                return doDelegateFunc<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10>(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11));
                return doDelegateFunc<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11>(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12));
                return doDelegateFunc<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12>(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13));
                return doDelegateFunc<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13>(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13);
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13, typename P14>
            void call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13, const P14& p14) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14));
                doDelegateFunc(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14);
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13, typename P14>
            R call(const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13, const P14& p14) {
                std::string func_name = "";
                {
                    rbk::lockGuard l(_bind_mutex);
                    func_name = _delegate_func_name;
                }
                LogInfo(formatLog("Delegate", _delegate_name, _device_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14));
                return doDelegateFunc<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14>(func_name, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14);
            }

            template<typename R>
            R doDelegateFunc(const std::string& func_name) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                int ret = lua_resume(_lua_state, 0);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0>
            void doDelegateFunc(const std::string& func_name, const P0& p0) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                int ret = lua_resume(_lua_state, 1);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0>
            R doDelegateFunc(const std::string& func_name, const P0& p0) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                int ret = lua_resume(_lua_state, 1);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                int ret = lua_resume(_lua_state, 2);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                int ret = lua_resume(_lua_state, 2);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                int ret = lua_resume(_lua_state, 3);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                int ret = lua_resume(_lua_state, 3);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                int ret = lua_resume(_lua_state, 4);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                int ret = lua_resume(_lua_state, 4);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                int ret = lua_resume(_lua_state, 5);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                int ret = lua_resume(_lua_state, 5);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                int ret = lua_resume(_lua_state, 6);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                int ret = lua_resume(_lua_state, 6);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                int ret = lua_resume(_lua_state, 7);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                int ret = lua_resume(_lua_state, 7);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                int ret = lua_resume(_lua_state, 8);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                int ret = lua_resume(_lua_state, 8);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                int ret = lua_resume(_lua_state, 9);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                int ret = lua_resume(_lua_state, 9);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                int ret = lua_resume(_lua_state, 10);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                int ret = lua_resume(_lua_state, 10);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                rbk::lua::pushLuaStackTraits(_lua_state, p10);
                int ret = lua_resume(_lua_state, 11);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                rbk::lua::pushLuaStackTraits(_lua_state, p10);
                int ret = lua_resume(_lua_state, 11);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                rbk::lua::pushLuaStackTraits(_lua_state, p10);
                rbk::lua::pushLuaStackTraits(_lua_state, p11);
                int ret = lua_resume(_lua_state, 12);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                rbk::lua::pushLuaStackTraits(_lua_state, p10);
                rbk::lua::pushLuaStackTraits(_lua_state, p11);
                int ret = lua_resume(_lua_state, 12);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                rbk::lua::pushLuaStackTraits(_lua_state, p10);
                rbk::lua::pushLuaStackTraits(_lua_state, p11);
                rbk::lua::pushLuaStackTraits(_lua_state, p12);
                int ret = lua_resume(_lua_state, 13);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                rbk::lua::pushLuaStackTraits(_lua_state, p10);
                rbk::lua::pushLuaStackTraits(_lua_state, p11);
                rbk::lua::pushLuaStackTraits(_lua_state, p12);
                int ret = lua_resume(_lua_state, 13);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                rbk::lua::pushLuaStackTraits(_lua_state, p10);
                rbk::lua::pushLuaStackTraits(_lua_state, p11);
                rbk::lua::pushLuaStackTraits(_lua_state, p12);
                rbk::lua::pushLuaStackTraits(_lua_state, p13);
                int ret = lua_resume(_lua_state, 14);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                rbk::lua::pushLuaStackTraits(_lua_state, p10);
                rbk::lua::pushLuaStackTraits(_lua_state, p11);
                rbk::lua::pushLuaStackTraits(_lua_state, p12);
                rbk::lua::pushLuaStackTraits(_lua_state, p13);
                int ret = lua_resume(_lua_state, 14);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13, typename P14>
            void doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13, const P14& p14) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                rbk::lua::pushLuaStackTraits(_lua_state, p10);
                rbk::lua::pushLuaStackTraits(_lua_state, p11);
                rbk::lua::pushLuaStackTraits(_lua_state, p12);
                rbk::lua::pushLuaStackTraits(_lua_state, p13);
                rbk::lua::pushLuaStackTraits(_lua_state, p14);
                int ret = lua_resume(_lua_state, 15);
                if (ret == 0) {
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    // TODO 错误处理，应该throw出来
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13, typename P14>
            R doDelegateFunc(const std::string& func_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13, const P14& p14) {
                rbk::lockGuard l(_lua_mutex);
                lua_getglobal(_lua_state, func_name.c_str());
                rbk::lua::pushLuaStackTraits(_lua_state, p0);
                rbk::lua::pushLuaStackTraits(_lua_state, p1);
                rbk::lua::pushLuaStackTraits(_lua_state, p2);
                rbk::lua::pushLuaStackTraits(_lua_state, p3);
                rbk::lua::pushLuaStackTraits(_lua_state, p4);
                rbk::lua::pushLuaStackTraits(_lua_state, p5);
                rbk::lua::pushLuaStackTraits(_lua_state, p6);
                rbk::lua::pushLuaStackTraits(_lua_state, p7);
                rbk::lua::pushLuaStackTraits(_lua_state, p8);
                rbk::lua::pushLuaStackTraits(_lua_state, p9);
                rbk::lua::pushLuaStackTraits(_lua_state, p10);
                rbk::lua::pushLuaStackTraits(_lua_state, p11);
                rbk::lua::pushLuaStackTraits(_lua_state, p12);
                rbk::lua::pushLuaStackTraits(_lua_state, p13);
                rbk::lua::pushLuaStackTraits(_lua_state, p14);
                int ret = lua_resume(_lua_state, 15);
                if (ret == 0) {
                    R r;
                    boost::fusion::for_each(r, boost::bind<void>(popLuaStateWrapper(), _lua_state, boost::placeholders::_1));
                    // clear lua_State
                    lua_settop(_lua_state, 0);
                    return r;
                }
                else {
                    LogFatal(formatLog("Delegate", _delegate_name, _device_name, std::string("Run delegate function error: ").append(lua_tostring(_lua_state, lua_gettop(_lua_state)))));
                    rbk::ErrorCodes::Instance()->setFatal(50002); // 内部 lua 调用出错
                    lua_settop(_lua_state, 0);
                    return R();
                    // TODO 错误处理，应该throw出来
                }
            }
        };
    } // namespace core
} // namespace rbk

#endif // ~_RBK_CORE_DELEGATE_H_
