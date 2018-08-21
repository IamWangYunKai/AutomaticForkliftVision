cmake_minimum_required(VERSION 3.5.2)

set(CMAKE_GENERATOR_TOOLSET "host=x64")

# WIN32 means windows, including Win64
# APPLE means macOS
# UNIX means Unix like system

# CMAKE PARAMS for RoboKit
# RBK_WITHOUT_SIM: whether to build without PioneerSim plugins, default is OFF. i.e: cmake -DRBK_WITHOUT_SIM=ON
# RBK_WITH_ALGORITHMS: whether to build with algorithms, default is ON. i.e: cmake -DRBK_WITH_ALGORITHMS=OFF
# RBK_WITH_PLUGINS: whehter to build with plugins, default is ON. i.e: cmake -DRBK_WITH_PLUGINS=OFF
# RBK_ENABLE_CPP11: build RoboKit with C++11 features enabled, default is ON. i.e: cmake -DRBK_ENABLE_CPP11=OFF
# RBK_BUILD_EXAMPLES: build RoboKit examples, default is OFF. i.e: cmake -DRBK_BUILD_EXAMPLES=OFF
# RBK_BUILD_TESTS: build RoboKit tests, default is OFF. i.e: cmake -DRBK_BUILD_TESTS=OFF

project(RoboKit)

# some paths
# ----------
list(APPEND CMAKE_MODULE_PATH ${RoboKit_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH ${RoboKit_SOURCE_DIR}/cmake)

# config file
# -----------
include(rbkConfig)

# global settings
# ---------------
include(GlobalSettings)

# Include our cmake macros
# ------------------------
include(CMakeHelpers)

# logging
# -------
include(CMakeLogging)

# build type (affects debugging symbols and optimization)
# -------------------------------------------------------
include(BuildType)

set(RBK_PATH "${RoboKit_SOURCE_DIR}" CACHE PATH "RoboKit SDK path" FORCE)

# Override from command line "CMake -D<OPTION>=TRUE/FALSE/0/1/ON/OFF"
option(RBK_WITHOUT_SIM "Whether to build without PioneerSim plugins" OFF)
option(RBK_WITH_ALGORITHMS "Whether to build with algorithms" ON)
option(RBK_WITH_PLUGINS "Whehter to build with plugins" ON)

option(RBK_BUILD_EXAMPLES "Build examples." OFF)
option(RBK_BUILD_TESTS "Build tests." OFF)
mark_as_advanced(RBK_BUILD_EXAMPLES)
mark_as_advanced(RBK_BUILD_TESTS)

message("RBK_WITHOUT_SIM set to " ${RBK_WITHOUT_SIM})
message("RBK_WITH_ALGORITHMS set to " ${RBK_WITH_ALGORITHMS})
message("RBK_WITH_PLUGINS set to " ${RBK_WITH_PLUGINS})

message("RBK_BUILD_EXAMPLES: " ${RBK_BUILD_EXAMPLES})
message("RBK_BUILD_TESTS: " ${RBK_BUILD_TESTS})

message("")

# skip install rules
# ------------------
# set(CMAKE_SKIP_INSTALL_RULES TRUE)

# compiler flags
# --------------
include(CompilerFlags)

# precompiled header
# ------------------
# include(cotire)

# MESSAGE("Looking for compiler:")
# --------------------------------
# INCLUDE (${CMAKE_ROOT}/Modules/CMakeDetermineCXXCompiler.cmake)
# include(${PROJ_SOURCE_DIR}/cmake/DependentOption.cmake)

# Shared or static library
# ------------------------
# IF (NOT LIBRARY_TYPE)
#   set(LIBRARY_TYPE shared cache string "What type of library to build: (shared, static or both)")
# ENDIF (NOT LIBRARY_TYPE)

# find libs
# ---------
message("Finding libraries:")
message("-----------------------------------------------------")
mkdir_if_not_exists(${RBK_THIRDPARTY_LIB_DIR})
include(GetThirdPartyLibs)

# default includes
# ----------------
include(DefaultIncludes)

# other settings
# --------------
include(RBKOtherSettings)

# other process
# -------------
# make log directory for logs
mkdir_if_not_exists(${RBK_OUTPUT_DIR}/log)

# options to build tests and examples
# -----------------------------------
# DEPENDENT_OPTION( BUILD_TESTS "Turn me off to disable the testing" ON "CPPUNIT_FOUND" OFF)
# OPTION( BUILD_EXAMPLES "Turn me off to disable the compilation of the examples" ON )

# Uninstall target
# ----------------
# CONFIGURE_FILE(
#   "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
#   "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
#   IMMEDIATE @ONLY)
# ADD_CUSTOM_TARGET(uninstall
#   "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

message("")

# add subfolders
# --------------
add_subdirs("src/robokit")

add_subdirectory(plugins)

# .pc file
# --------

# Building Doxygen documents
# --------------------------


# copy files
# ----------
# copy boost dll
IF(WIN32)
    IF(MSVC14)
        set(vc_version "vc140")
    ELSEIF(MSVC10)
        set(vc_version "vc100")
    ENDIF()

    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_atomic-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_atomic-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_chrono-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_chrono-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_date_time-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_date_time-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_filesystem-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_filesystem-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_log_setup-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_log_setup-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_log-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_log-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_random-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_random-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_regex-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_regex-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_system-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_system-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_thread-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_thread-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_timer-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_timer-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})
    copy_if_not_exists(${RBK_OUTPUT_DIR}/boost_program_options-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_BOOST_DIR}/lib/boost_program_options-${vc_version}-mt-${Boost_LIB_VERSION}.dll ${RBK_OUTPUT_DIR})

    unset(vc_version)
ELSE()
   # set(Boost_VERSION "1.63.0")
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_atomic-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_atomic-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_chrono-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_chrono-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_date_time-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_date_time-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_filesystem-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_filesystem-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_log_setup-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_log_setup-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_log-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_log-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_random-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_random-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_regex-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_regex-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_system-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_system-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_thread-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_thread-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_timer-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_timer-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # copy_if_not_exists(${RBK_OUTPUT_DIR}/libboost_program_options-mt.so.${Boost_VERSION} ${RBK_BOOST_DIR}/lib/libboost_program_options-mt.so.${Boost_VERSION} ${RBK_OUTPUT_DIR})
   # unset(Boost_VERSION)
ENDIF()

# copy lua dll to bin
IF(WIN32)
file(COPY ${LUA_DLL} DESTINATION ${RBK_OUTPUT_DIR})
ENDIF()

# copy protobuf so to bin
IF(UNIX)
file(COPY ${LIBPROTOBUF_DIR}/lib/libprotobuf.so
          DESTINATION ${RBK_OUTPUT_DIR})
file(COPY ${LIBPROTOBUF_DIR}/lib/libprotobuf.so.13
          DESTINATION ${RBK_OUTPUT_DIR})
file(COPY ${LIBPROTOBUF_DIR}/lib/libprotobuf.so.13.0.0
          DESTINATION ${RBK_OUTPUT_DIR})
ENDIF()

# copy sqlite3 dll to bin
IF(WIN32)
file(COPY ${SQLite3_DLL_RELEASE} DESTINATION ${RBK_OUTPUT_DIR})
ELSE()
file(COPY ${LIBSQLITE3_DIR}/lib/
          DESTINATION ${RBK_OUTPUT_DIR}
          FILES_MATCHING REGEX "libsqlite3.so*"
          PATTERN "pkgconfig" EXCLUDE
          PATTERN "libsqlite3.a" EXCLUDE)
ENDIF()

# copy proto to bin
file(COPY ${PROJ_SOURCE_DIR}/proto/
          DESTINATION ${RBK_OUTPUT_DIR}/proto)

