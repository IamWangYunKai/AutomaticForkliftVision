#ifndef _RBK_CORE_SINGLETON_H_
#define _RBK_CORE_SINGLETON_H_

#include <robokit/config.h>

namespace rbk {
    namespace core {
        /************************************************************************/
        /*            Meyers Singleton: Release resource before exiting			*/
        /*                                                                      */
        /* 1:优点:                                                              */
        /* 1）: 该实现是一个“懒汉”单例模式，意味着只有在第一次调用 Instance() */
        /*      的时候才会实例化                                                */
        /* 2）: 不需要每次调用 Instance() 静态方法时，必须判断 NULL==_instance，*/
        /*      效率相对高一点                                                  */
        /* 3）: 使用对象而不是指针分配内存，因此自动调用析构函数，不会导致内存  */
        /*      泄露                                                            */
        /* 4）: 在多线程下的确能够保证有且只有一个实例产生。                    */
        /*                                                                      */
        /* 2:缺点:                                                              */
        /* 在多线程情况下，并不是真正的线程安全                                 */
        /************************************************************************/
        template <typename SingletonClass>
        class MeyersSingleton {
        public:
            static SingletonClass* Instance() {
                static SingletonClass instance;
                return &instance;
            }

            SingletonClass* operator ->() { return Instance(); }

            const SingletonClass* operator ->() const { return Instance(); }

        private:
            MeyersSingleton();

            ~MeyersSingleton();

            MeyersSingleton(const MeyersSingleton&);

            MeyersSingleton& operator=(const MeyersSingleton&);
        };

        /************************************************************************/
        /*			Normal Singleton: Not Release resource before exiting		*/
        /************************************************************************/
        template <typename SingletonClass>
        class NormalSingleton {
        public:
            static SingletonClass* Instance() {
                static SingletonClass* instance = nullptr;
                if (!instance) {
                    instance = new SingletonClass;
                }
                return instance;
            }

            SingletonClass* operator ->() { return Instance(); }

            const SingletonClass* operator ->() const { return Instance(); }

        private:
            NormalSingleton();

            ~NormalSingleton();

            NormalSingleton(const NormalSingleton&);

            NormalSingleton& operator=(const NormalSingleton&);
        };

        /************************************************************************/
        /*			Boost Singleton: boost/container/detail/singleton.hpp		*/
        /************************************************************************/
        // SingletonClass must be: no-throw default constructible and no-throw destructible
        template <typename SingletonClass>
        class Singleton {
        private:
            struct object_creator {
                // This constructor does nothing more than ensure that Instance()
                // is called before main() begins, thus creating the static
                // SingletonClass object before multithreading race issues can come up.
                object_creator() { Singleton<SingletonClass>::Instance(); }
                inline void do_nothing() const { }
            };
            static object_creator create_object;

            Singleton();

            ~Singleton();

            Singleton(const Singleton&);

            Singleton& operator=(const Singleton&);

        public:
            typedef SingletonClass object_type;

            // If, at any point (in user code), Singleton<SingletonClass>::Instance()
            // is called, then the following function is instantiated.
            static object_type* Instance() {
                // This is the object that we return a reference to.
                // It is guaranteed to be created before main() begins because of
                // the next line.
                static object_type obj;

                // The following line does nothing else than force the instantiation
                // of Singleton<SingletonClass>::create_object, whose constructor is
                // called before main() begins.
                create_object.do_nothing();
                // C++ Standard 3.6.2 Initialization of non-local variables:
                // It is implementation-defined whether the dynamic initialization of a non-local variable with static storage
                // duration is done before the first statement of main...

                return &obj;
            }
        };
        template <typename SingletonClass>
        typename Singleton<SingletonClass>::object_creator Singleton<SingletonClass>::create_object;
    } // namespace core
} // namespace rbk

#endif	// ~_RBK_CORE_SINGLETON_H_
