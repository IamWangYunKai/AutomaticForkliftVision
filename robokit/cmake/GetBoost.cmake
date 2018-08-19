# boost >= 1.60.0

set(BOOST_ROOT ${RBK_BOOST_DIR})
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_NO_SYSTEM_PATHS ON)
set(Boost_DEBUG ON)
find_package(Boost 1.60.0 REQUIRED)
