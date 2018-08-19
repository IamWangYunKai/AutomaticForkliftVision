#ifndef _RBK_CORE_ATTRIBUTE_H_
#define _RBK_CORE_ATTRIBUTE_H_

#include <robokit/config.h>

namespace rbk {
    namespace core {
        class RBK_API BaseAttribute {
        public:
            BaseAttribute() {
            }
            virtual ~BaseAttribute() {}
        protected:
            std::string m_attribute;
        };

        template<typename T>
        class Attribute : BaseAttribute {
        public:
            Attribute(std::string name) {
                m_attribute = name;
            }
            ~Attribute() {}
            void set(T v) {
                m_mutex.lock();
                m_value = v;
                m_mutex.unlock();
            }
            T get() {
                T v;
                m_mutex.lock();
                v = m_value;
                m_mutex.unlock();
                return v;
            }
        private:
            T m_value;
            std::mutex m_mutex;
        };
    } // namespace core
} // namespace rbk

#endif // ~_RBK_CORE_ATTRIBUTE_H_
