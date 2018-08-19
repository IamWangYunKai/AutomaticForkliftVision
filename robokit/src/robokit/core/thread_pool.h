#ifndef _RBK_THREAD_POOL_H_
#define _RBK_THREAD_POOL_H_

#include <robokit/config.h>
#include <robokit/core/threadpool.hpp>
#include <robokit/core/logger.h>

namespace rbk {
    class RBK_API ThreadPool {
    private:
        struct object_creator {
            // This constructor does nothing more than ensure that Instance()
            // is called before main() begins, thus creating the static
            // SingletonClass object before multithreading race issues can come up.
            object_creator() { ThreadPool::Instance(); }
            inline void do_nothing() const { }
        };

        static object_creator create_object;

        ThreadPool();

        ~ThreadPool();

        ThreadPool(const ThreadPool&);

        ThreadPool& operator=(const ThreadPool&);

    public:

        static ThreadPool* Instance();

        void wait();

        void resize(uint8_t threadNum);

        size_t size();

        size_t active();

        template<typename CallBack>
        bool schedule(CallBack f) {
            // spin lock
            while (_syncFlag.test_and_set());

            if (_pool.size() == _pool.active()) {
                size_t poolSize = _pool.size() + RBK_DEFAULT_THREADPOOL_STEP;
                if (poolSize > RBK_MAX_THREADPOOL_SIZE) {
                    LogError(formatLogText("Thread pool is full, max size is " << RBK_MAX_THREADPOOL_SIZE));
                }
                else {
                    _pool.size_controller().resize(poolSize);
                }
            }
            _syncFlag.clear();
            return _pool.schedule(f);
        }

    private:
        boost::threadpool::thread_pool<boost::threadpool::task_func,
            boost::threadpool::fifo_scheduler,
            boost::threadpool::static_size,
            boost::threadpool::resize_controller,
            boost::threadpool::immediately> _pool;
        std::atomic_flag _syncFlag = ATOMIC_FLAG_INIT;
    };
}

#endif // ~_RBK_THREAD_POOL_H_
