#ifndef _RBK_MUTEX_H_
#define _RBK_MUTEX_H_

#include <robokit/config.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/lock_types.hpp>
#include <boost/thread/lock_guard.hpp>

namespace rbk {
    using readLock = boost::shared_lock<boost::shared_mutex>;
    using writeLock = boost::unique_lock<boost::shared_mutex>;
    using upgradeLock = boost::upgrade_lock<boost::shared_mutex>;
    using upgradeToUniqueLock = boost::upgrade_to_unique_lock<boost::shared_mutex>;
    using lock = boost::unique_lock<boost::mutex>;
    using lockGuard = boost::lock_guard<boost::mutex>;
    using rwMutex = boost::shared_mutex;
    using sharedMutex = boost::shared_mutex;
    using mutex = boost::mutex;
} // namespace rbk

#endif // ~_RBK_MUTEX_H_
