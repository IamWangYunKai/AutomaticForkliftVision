#ifndef _RBK_CORE_SUBSCRIBER_H_
#define _RBK_CORE_SUBSCRIBER_H_

#include <robokit/config.h>

#include <mutex>
#include <functional>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

namespace rbk {
    namespace core {
        class RBK_API BaseSubscriber {
        public:
            BaseSubscriber() {}
            virtual void close() {}
            std::string topic() const { return m_topic; }
        protected:
            std::string m_topic;
        };

        class Recorder;
        class RBK_API Subscriber : public BaseSubscriber {
        public:
            Subscriber(const std::string& topic, const std::string& name);
            ~Subscriber();

            template <typename T, typename CallBack>
            void init(CallBack globalFunc) {
                if (m_initialized) {
                    return;
                }

                m_msg = new T;
                m_tmp_msg = new T;
                if (globalFunc != NULL) {
                    m_func = globalFunc;
                }
                m_initialized = true;
            }

            template <typename T, typename CallBack, typename T1>
            void init(CallBack memberFunc, T1 *pThis) {
                if (m_initialized) {
                    return;
                }

                m_msg = new T;
                m_tmp_msg = new T;
                m_func = std::bind(memberFunc, pThis, std::placeholders::_1);
                m_initialized = true;
            }

            template <typename T>
            void init() {
                if (m_initialized) {
                    return;
                }

                m_msg = new T;
                m_tmp_msg = new T;
                m_func = NULL;
                m_initialized = true;
            }

            void init(std::function<void(google::protobuf::Message*)> globalFunc);

            template<typename T>
            void UpdateData(T& data) {
                m_mutex.lock();
                m_msg->CopyFrom(data);
                m_mutex.unlock();
                if (NULL != m_func) {
                    m_func(&data);
                }
            }

            template<typename T>
            void UpdateMap(T& data) {
                m_mutex.lock();
                if (m_msg != NULL) {
                    delete m_msg;
                    m_msg = NULL;
                }
                m_msg = new T;
                m_msg->CopyFrom(data);
                m_mutex.unlock();
                if (NULL != m_func) {
                    m_func(&data);
                }
            }

            template<typename T>
            void GetData(T& data) {
                m_mutex.lock();
                data.CopyFrom(*m_msg);
                m_mutex.unlock();
            }

            void UpdateDataInString(const std::string& data);

            std::string name() const;
        private:
            bool m_initialized;
            bool m_need_record_log;
            std::string m_name;
            std::function<void(google::protobuf::Message*)> m_func;
            google::protobuf::Message* m_msg;
            google::protobuf::Message* m_tmp_msg;
            std::mutex m_mutex;
        };
    } // namespace core
} // namespace rbk

#endif // ~_RBK_CORE_SUBSCRIBER_H_
