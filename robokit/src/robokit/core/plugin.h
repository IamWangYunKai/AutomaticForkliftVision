#ifndef _RBK_CORE_PLUGIN_H_
#define _RBK_CORE_PLUGIN_H_

#include <string>
#include <vector>
#include <map>

#include <robokit/config.h>
#include <robokit/core/publisher.h>
#include <robokit/core/subscriber.h>
#include <robokit/core/service.h>
#include <robokit/core/attribute.h>
#include <robokit/core/event.h>
#include <robokit/core/delegate.h>
#include <robokit/core/lua_reader.h>
#include <robokit/core/logger.h>
#include <robokit/core/sleep.h>
#include <robokit/core/param.h>
#include <robokit/core/luawapper.h>


namespace rbk {
    namespace protocol {
        class Message_Action;
        class Message_ActionResult;
    };
    namespace core {
        class Action;
        class RBK_API NPlugin {
        private:
            NPlugin(const NPlugin&);
            NPlugin& operator=(const NPlugin&);
        protected:
            std::string m_device_name;
            std::string m_original_device_name;

            std::map<std::string, Publisher*> m_publishers;
            std::map<std::string, Subscriber*> m_subscribers;
            std::map<std::string, BaseService*> m_services;
            std::map<std::string, BaseAttribute*> m_attributes;
            std::map<std::string, Event*> m_events;
            std::map<std::string, Delegate*> m_delegates;

            Params _params;

            lua_State* m_lua_state;
            lua_State* m_service_thread;

            rbk::mutex m_service_mutex;

            std::atomic<bool> _initialized = false;

        public:
            NPlugin();
            NPlugin(const std::string& name);
            virtual void loadFromConfigFile() = 0;
            virtual void setSubscriberCallBack() = 0;
            virtual void run() = 0;
            void start();
            void initialize();
            std::string name() const;
            std::string originalName() const;
            void setOriginalName(const std::string& name);
            void setName(const std::string& name);
            void setLuaState(lua_State* s);

            Subscriber* getSubscriber(const std::string& topic) const;
            Publisher* getPublisher(const std::string& topic);
            void connectTo(const NPlugin* recv_dev, const std::string& topic);
            void disconnectTo(const NPlugin* recv_dev, const std::string& topic);
            void createPublisher(const std::string& topic);
        
            Action* createAction(std::string name, std::function<void(rbk::protocol::Message_Action*)> func);
            void finishAction(std::string name, rbk::protocol::Message_ActionResult* res = NULL);

            template<typename T>
            void finishAction(std::string name, T& res) {
                ActionManager::Instance()->finishAction(m_device_name, name, res);
            }

            void cancelAction(std::string name);
            void failAction(std::string name);
            void suspendAction(std::string name);
            void resumeAction(std::string name);
            void setActionResult(std::string name, rbk::protocol::Message_ActionResult*);

            template<typename T>
            void loadParam(MutableParam<T>& param, const std::string& key, T defaultValue, T minValue = std::numeric_limits<T>::lowest(), T maxValue = std::numeric_limits<T>::max(), const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", bool advanced = false) {
                T value;
                std::string errMsg;
                if (_params.findParam(key, value, defaultValue, true)) {
                    if (!param.init(key, value, &_params, defaultValue, minValue, maxValue, group, desc, advanced, errMsg)) {
                        LogError(formatLogText("loadParam: " << m_device_name << " failed to load mutable param \"" << key << "\", " << errMsg));
                    }
                }
                else {
                    LogWarn(formatLogText("loadParam: " << m_device_name << " failed to load mutable param \"" << key << "\", use default value ( " << defaultValue << " )"));
                    if (!param.init(key, defaultValue, &_params, defaultValue, minValue, maxValue, group, desc, advanced, errMsg)) {
                        LogError(formatLogText("loadParam: " << m_device_name << " failed to load mutable param \"" << key << "\", " << errMsg));
                    }
                }
            }

            void loadParam(MutableParam<bool>& param, const std::string& key, bool defaultValue, const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", bool advanced = false);
            void loadParam(MutableParam<std::string>& param, const std::string& key, const std::string& defaultValue, const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", bool advanced = false);

            template<typename T>
            void loadParam(Param<T>& param, const std::string& key, T defaultValue, T minValue = std::numeric_limits<T>::lowest(), T maxValue = std::numeric_limits<T>::max(), const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", bool advanced = false) {
                T value;
                std::string errMsg;
                if (_params.findParam(key, value, defaultValue, false)) {
                    if (!param.init(key, value, defaultValue, minValue, maxValue, group, desc, advanced, errMsg)) {
                        LogError(formatLogText("loadParam: " << m_device_name << " failed to load param \"" << key << "\", " << errMsg));
                    }
                }
                else {
                    LogWarn(formatLogText("loadParam: " << m_device_name << " failed to load param \"" << key << "\", use default value ( " << defaultValue << " )"));
                    if (!param.init(key, defaultValue, defaultValue, minValue, maxValue, group, desc, advanced, errMsg)) {
                        LogError(formatLogText("loadParam: " << m_device_name << " failed to load param \"" << key << "\", " << errMsg));
                    }
                }
            }

            void loadParam(Param<bool>& param, const std::string& key, bool defaultValue, const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", bool advanced = false);
            void loadParam(Param<std::string>& param, const std::string& key, const std::string& defaultValue, const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", bool advanced = false);

            template<typename T>
            void loadParam(T& param, const std::string& key, T defaultValue, T minValue = std::numeric_limits<T>::lowest(), T maxValue = std::numeric_limits<T>::max(), const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", typename boost::enable_if<boost::mpl::or_<boost::is_integral<T>, boost::is_floating_point<T>>>::type* dummy = 0) {
                if (!_params.findParam(key, param, defaultValue, false)) {
                    param = defaultValue;
                    LogWarn(formatLogText("loadParam: " << m_device_name << " failed to load param \"" << key << "\", use default value ( " << defaultValue << " )"));
                }
            }

            void loadParam(bool& param, const std::string& key, bool defaultValue, const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "");
            void loadParam(std::string& param, const std::string& key, const std::string& defaultValue, const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "");

            void saveParam();

            void reloadParam();

            bool isParamChanged();

            template<typename T>
            void publishTopic(T& data) {
                std::string topic = T::descriptor()->full_name();
                std::map<std::string, Publisher*>::const_iterator iter = m_publishers.find(topic);
                if (m_publishers.end() == iter) {
                    LogWarn(formatLog("PubSub", topic, m_device_name, "plugin has not created publisher for this topic"));
                }
                else {
                    Publisher* p = iter->second;
                    if (topic == "rbk.protocol.Message_Map") {
                        p->publishMap(data);
                    }
                    else {
                        p->publish(data);
                    }
                }
            }

            //template<typename T>
            //void recordTopic(const T& data) {
            //    google::protobuf::util::JsonOptions json_options;
            //    //json_options.always_print_primitive_fields = true;
            //    std::string json_buffer;
            //    google::protobuf::util::Status convertStatus = google::protobuf::util::MessageToJsonString(data, &json_buffer, json_options);
            //    if (convertStatus.ok()) {
            //        LogRecord(T::descriptor()->full_name(), json_buffer);
            //    }
            //    else {
            //        LogError("recordTopic: MessageToJsonString for " << T::descriptor()->full_name() << " is failed, " << convertStatus.ToString());
            //    }
            //}

            template<typename T>
            void getSubscriberData(T& value) {
                std::string topic = T::descriptor()->full_name();
                std::map<std::string, Subscriber*>::const_iterator iter = m_subscribers.find(topic);
                if (m_subscribers.end() == iter) {
                    LogWarn(formatLog("PubSub", topic, m_device_name, "plugin has not subscribed this topic"));
                    return;
                }
                iter->second->GetData(value);
            }

            template<typename T>
            void createPublisher() {
                std::string topic = T::descriptor()->full_name();
                std::map<std::string, Publisher*>::const_iterator iter = m_publishers.find(topic);
                if (m_publishers.end() == iter) {
                    Publisher* p = new Publisher(topic, m_device_name);
                    m_publishers.insert(std::make_pair(topic, p));
                }
                else {
                    LogWarn(formatLog("PubSub", topic, m_device_name, "publisher has been created for this topic"));
                }
            }

            template<typename T, typename CallBack>
            void setTopicCallBack(CallBack f) {
                std::string topic = T::descriptor()->full_name();
                std::map<std::string, Subscriber*>::const_iterator iter = m_subscribers.find(topic);
                if (m_subscribers.end() == iter) {
                    Subscriber* s = new Subscriber(topic, m_device_name);
                    s->init<T>(f);
                    m_subscribers.insert(std::make_pair(topic, s));

                    if (m_device_name == "MoveFactory" && topic == "rbk.protocol.Message_MoveTask") {
                        std::string service_name("update_");
                        service_name.append(topic);
                        setServiceCallBack<void(std::string)>(service_name, &Subscriber::UpdateDataInString, s);
                    }
                }
                else {
                    LogWarn(formatLog("PubSub", topic, m_device_name, "subscriber has been created for this topic"));
                }
            }

            template<typename T>
            void setTopicCallBack() {
                std::string topic = T::descriptor()->full_name();
                std::map<std::string, Subscriber*>::const_iterator iter = m_subscribers.find(topic);
                if (m_subscribers.end() == iter) {
                    Subscriber* s = new Subscriber(topic, m_device_name);
                    s->init<T>();
                    m_subscribers.insert(std::make_pair(topic, s));

                    if (m_device_name == "MoveFactory" && topic == "rbk.protocol.Message_MoveTask") {
                        std::string service_name("update_");
                        service_name.append(topic);
                        setServiceCallBack<void(std::string)>(service_name, &Subscriber::UpdateDataInString, s);
                    }
                }
                else {
                    LogWarn(formatLog("PubSub", topic, m_device_name, "subscriber has been created for this topic"));
                }
            }

            template<typename T, typename CallBack, typename T2>
            void setTopicCallBack(CallBack f, T2* pthis) {
                std::string topic = T::descriptor()->full_name();
                std::map<std::string, Subscriber*>::const_iterator iter = m_subscribers.find(topic);
                if (m_subscribers.end() == iter) {
                    Subscriber* s = new Subscriber(topic, m_device_name);
                    s->init<T>(f, pthis);
                    m_subscribers.insert(std::make_pair(topic, s));

                    if (m_device_name == "MoveFactory" && topic == "rbk.protocol.Message_MoveTask") {
                        std::string service_name("update_");
                        service_name.append(topic);
                        setServiceCallBack<void(std::string)>(service_name, &Subscriber::UpdateDataInString, s);
                    }
                }
                else {
                    LogWarn(formatLog("PubSub", topic, m_device_name, "subscriber has been created for this topic"));
                }
            }

            template<typename Signature, typename CallBack, typename T2>
            void setServiceCallBack(const std::string& service, CallBack f, T2* pthis) {
                std::map<std::string, BaseService*>::const_iterator iter = m_services.find(service);
                if (m_services.end() == iter) {
                    Service<Signature>* s = new Service<Signature>(service, m_lua_state);
                    s->init(f, pthis);
                    m_services.insert(std::make_pair(service, s));
                    LogInfo(formatLog("Service", service, m_device_name, "create"));
                }
                else {
                    LogWarn(formatLog("Service", service, m_device_name, "has been created"));
                }
            }

            template<typename ReturnT, typename... Ps>
            ReturnT callService(const std::string& pluginName, const std::string& serviceName, const Ps&... ps) {
                rbk::lockGuard l(m_service_mutex);

                rbk::core::lua::State ss(m_service_thread);

                if (ss["rbk"][pluginName.c_str()].isNilref()) {
                    LogError(formatLog("Service", serviceName, pluginName, "plugin is invalid(call from C++)"));
                    return ReturnT();
                }
                else {
                    LogInfo(formatLog("Service", serviceName, pluginName, "(call from C++)"));
                    try {
                        if (ss["rbk"][pluginName.c_str()][serviceName.c_str()].isNilref()) {
                            LogError(formatLog("Service", serviceName, pluginName, "plugin does not have this service(call from C++)"));
                            return ReturnT();
                        }
                        else {
                            return ss["rbk"][pluginName.c_str()][serviceName.c_str()].call<ReturnT, const Ps&...>(ps...);
                        }
                    }
                    catch (std::exception& e) {
                        LogError(formatLog("Service", serviceName, pluginName, std::string(e.what()).append("(call from C++)")));
                        return ReturnT();
                    }
                    catch (...) {
                        LogError(formatLog("Service", serviceName, pluginName, std::string("lua status ").append(std::to_string(lua_status(m_service_thread))).append("(call from C++)")));
                        return ReturnT();
                    }
                }
            }

            RBK_DEPRECATED void createPluginEvent(const std::string& event_name);

            template <typename T>
            void fireEvent(const T event_name) {
                std::map<std::string, Event*>::iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    Event* e = iter->second;
                    e->fire<T>();
                }
            }

            void bindEvent(const std::string& event_name, const std::string& func_name);

            void unbindEvent(const std::string& event_name, const std::string& func_name);

            void createEvent(const std::string& event_name);

            void fireEvent(const std::string& event_name);

            template<typename P0>
            void fireEvent(const std::string& event_name, const P0& p0) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0>(p0);
                }
            }

            template<typename P0, typename P1>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1>(p0, p1);
                }
            }

            template<typename P0, typename P1, typename P2>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2>(p0, p1, p2);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3>(p0, p1, p2, p3);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4>(p0, p1, p2, p3, p4);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4, P5>(p0, p1, p2, p3, p4, p5);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
            void fireEvent(const std::string& event_name, const  P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4, P5, P6>(p0, p1, p2, p3, p4, p5, p6);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4, P5, P6, P7>(p0, p1, p2, p3, p4, p5, p6, p7);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4, P5, P6, P7, P8>(p0, p1, p2, p3, p4, p5, p6, p7, p8);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13, typename P14>
            void fireEvent(const std::string& event_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13, const P14& p14) {
                std::map<std::string, Event*>::const_iterator iter = m_events.find(event_name);
                if (m_events.end() == iter) {
                    LogWarn(formatLog("Event", event_name, m_device_name, "no this event"));
                }
                else {
                    iter->second->fire<P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14);
                }
            }

            void bindDelegate(const std::string& delegate_name, const std::string& func_name);

            void unbindDelegate(const std::string& delegate_name, const std::string& func_name);

            template<typename T>
            void callDelegate(const T delegate_name) {
                std::map<std::string, Delegate*>::iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    Delegate* d = iter->second;
                    d->call<boost::tuple<>>(); // trick
                }
            }

            bool delegateLuaFuncExists(const std::string& delegate_name);

            void createDelegate(const std::string& delegate_name);

            // R 里面不能用指针类型，如 char*, char* 返回后仍然是 nullptr
            template<typename R>
            R callDelegate(const std::string& delegate_name) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R>();
                }
            }

            template<typename P0>
            void callDelegate(const std::string& delegate_name, const P0& p0) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0);
                }
            }

            template<typename R, typename P0>
            R callDelegate(const std::string& delegate_name, const P0& p0) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0>(p0);
                }
            }

            template<typename P0, typename P1>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1);
                }
            }

            template<typename R, typename P0, typename P1>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1>(p0, p1);
                }
            }

            template<typename P0, typename P1, typename P2>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2);
                }
            }

            template<typename R, typename P0, typename P1, typename P2>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2>(p0, p1, p2);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3>(p0, p1, p2, p3);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4>(p0, p1, p2, p3, p4);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4, p5);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4, P5>(p0, p1, p2, p3, p4, p5);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4, p5, p6);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4, P5, P6>(p0, p1, p2, p3, p4, p5, p6);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4, p5, p6, p7);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4, P5, P6, P7>(p0, p1, p2, p3, p4, p5, p6, p7);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4, p5, p6, p7, p8);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4, P5, P6, P7, P8>(p0, p1, p2, p3, p4, p5, p6, p7, p8);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13);
                }
            }

            template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13, typename P14>
            void callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13, const P14& p14) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                }
                else {
                    iter->second->call(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14);
                }
            }

            template<typename R, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8, typename P9, typename P10, typename P11, typename P12, typename P13, typename P14>
            R callDelegate(const std::string& delegate_name, const P0& p0, const P1& p1, const P2& p2, const P3& p3, const P4& p4, const P5& p5, const P6& p6, const P7& p7, const P8& p8, const P9& p9, const P10& p10, const P11& p11, const P12& p12, const P13& p13, const P14& p14) {
                std::map<std::string, Delegate*>::const_iterator iter = m_delegates.find(delegate_name);
                if (m_delegates.end() == iter) {
                    LogWarn(formatLog("Delegate", delegate_name, m_device_name, "no this delegate"));
                    return R();
                }
                else {
                    return iter->second->call<R, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14>(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14);
                }
            }
        };
    } // namespace core
} // namespace rbk

#endif // ~_RBK_CORE_PLUGIN_H_
