#ifndef _RBK_CORE_PUBLISHER_H_
#define _RBK_CORE_PUBLISHER_H_

#include <robokit/config.h>

#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/type_resolver_util.h>
#include <robokit/core/subscriber.h>
#include <robokit/core/thread_pool.h>

namespace rbk {
    namespace core {
        class Subscriber;
        class RBK_API BasePublisher {
        public:
            BasePublisher() {}
            virtual void close() {}
        protected:
            std::string m_topic;
        };

        class RBK_API Publisher : public BasePublisher {
        public:
            Publisher(std::string topic, std::string name) {
                m_topic = topic;
                m_initialized = false;
                m_name = name;
            };

            template<typename T>
            void publish(T& data) {
                m_sub_mutex.lock();
                for (std::vector<Subscriber*>::iterator iter = m_subscribers.begin(); iter != m_subscribers.end(); ++iter) {
                    //rbk::ThreadPool::Instance()->schedule(std::bind(&Subscriber::UpdateData<T>, *iter, data));
                    // TODO 这里比上面一行写法快 100 倍, protobuf 占用的内存小很多, 但是还可以再优化
                    (*iter)->UpdateData(data);
                }
                m_sub_mutex.unlock();
            }

            template<typename T>
            void publishMap(T& data) {
                m_sub_mutex.lock();
                for (std::vector<Subscriber*>::iterator iter = m_subscribers.begin(); iter != m_subscribers.end(); ++iter) {
                    (*iter)->UpdateMap(data);
                }
                m_sub_mutex.unlock();
            }

            template<typename T>
            void publish(const T& data, const std::string& recv_dev) {
                m_sub_mutex.lock();
                for (std::vector<Subscriber*>::iterator iter = m_subscribers.begin(); iter != m_subscribers.end(); ++iter) {
                    if (recv_dev == (*iter)->name()) {
                        (*iter)->UpdateData(data);
                        break;
                    }
                }
                m_sub_mutex.unlock();
            }

            bool addSubscriber(Subscriber* sub);
            bool removeSubscriber(Subscriber* sub);

        private:
            bool m_initialized;
            std::vector<Subscriber*> m_subscribers;
            std::mutex m_sub_mutex;
            std::string m_name;
        };
    } // namespace core
} // namespace rbk

#endif // ~_RBK_CORE_PUBLISHER_H_
