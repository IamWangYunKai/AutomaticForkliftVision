#ifndef _RBK_CHRONO_TIMER_H_
#define _RBK_CHRONO_TIMER_H_

#include <robokit/config.h>

#include <boost/asio/high_resolution_timer.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include <chrono>

namespace rbk {
    namespace chrono {
        template <typename Clock>
        class Timer {
        public:
            Timer() : _tBegin(typename Clock::now()) {}

            void reset() {
                _tBegin = typename Clock::now();
            }

            template <typename Duration>
            int64_t elapsed() {
                return std::chrono::duration_cast<Duration>((typename Clock::now()) - _tBegin).count();
            }

        private:
            Timer(const Timer&);
            Timer& operator=(const Timer&);

            typename Clock::time_point _tBegin;
        };

        class RBK_API WaitTimer {
        public:
            WaitTimer();
            ~WaitTimer();

            void wait();
            void wait(boost::system::error_code& ec);

            void asyncWait(const boost::function<void(const boost::system::error_code& ec)>& handler);

            boost::asio::high_resolution_timer::duration expireFromNow() const;
            size_t expireFromNow(const boost::asio::high_resolution_timer::duration& expiryTime);
            size_t expireFromNow(const boost::asio::high_resolution_timer::duration& expiryTime,
                boost::system::error_code& ec);

            boost::asio::high_resolution_timer::time_point expireAt() const;
            size_t expireAt(const boost::asio::high_resolution_timer::time_point& expiryTime);
            size_t expireAt(const boost::asio::high_resolution_timer::time_point& expiryTime,
                boost::system::error_code& ec);

            size_t cancel();
            size_t cancel(boost::system::error_code& ec);

            size_t cancelOne();
            size_t cancelOne(boost::system::error_code& ec);

        private:
            WaitTimer(const WaitTimer&);
            WaitTimer& operator=(const WaitTimer&);

            boost::asio::io_service _io;
            boost::asio::io_service::work _work;
            boost::asio::high_resolution_timer _timer;
            boost::thread _backgroundThread;
        };

        class RBK_API IntervalTimer {
        public:
            IntervalTimer();
            ~IntervalTimer();

            void setInterval(const boost::asio::high_resolution_timer::duration& intervalTime,
                const boost::function<void()>& cb);

            void setInterval(const boost::asio::high_resolution_timer::duration& intervalTime,
                const boost::function<void()>& cb,
                boost::system::error_code& ec);

            void clearInterval();
            void clearInterval(boost::system::error_code& ec);

        private:
            IntervalTimer(const IntervalTimer&);
            IntervalTimer& operator=(const IntervalTimer&);

            void internalCallback(const boost::asio::high_resolution_timer::duration& intervalTime,
                const boost::system::error_code& ec);

            boost::asio::io_service _io;
            boost::asio::io_service::work _work;
            boost::asio::high_resolution_timer _timer;
            boost::thread _backgroundThread;

            boost::function<void()> _callback;
        };
    } // namespace time
} // namespace rbk

#endif // ~_RBK_CHRONO_TIMER_H_
