# project version
# ---------------
include(version)

message("RoboKit v${ROBOKIT_VERSION}, © 2015-2018 Seer Robotics Co,.Ltd.")
message("Auther: Ye Yangsheng")
message("Email: yys@seer-robotics.com")
message("Website: http://www.seer-robotics.com")
message("=====================================================")

set(RBK_THIRDPARTY_LIB_DIR "D:/rbk_thirdparty_lib" CACHE PATH "RoboKit third party libraries path")

set(RBK_PATH "" CACHE PATH "RoboKit SDK path")

# set boost path
set(RBK_BOOST_DIR "D:/rbk_thirdparty_lib/boost_1_63_0" CACHE PATH "Boost directory for RoboKit")

# set QT path
set(RBK_QT_DIR "E:/Qt/Qt5.6.2_msvc2015/5.6/msvc2015/lib/cmake" CACHE PATH "Qt directory for RoboKit")

IF(NOT RBK_PLUGIN)
    IF(WIN32 AND CMAKE_CL_64)
        set(RBK_OUTPUT_DIR "${RoboKit_SOURCE_DIR}/bin64" CACHE PATH "output directory for RoboKit")
    ELSE()
        set(RBK_OUTPUT_DIR "${RoboKit_SOURCE_DIR}/bin" CACHE PATH "output directory for RoboKit")
    ENDIF()
ENDIF()

# set proto files' path
set(PROTO_DIR "${RBK_PATH}/proto")
set(RBK_PROTO_DIR ${PROTO_DIR} CACHE PATH "proto files' path" FORCE)
unset(PROTO_DIR)

# thirdparty lib name config
IF(CMAKE_CROSSCOMPILING)
    # cross compile for armv7l
    set(RBK_LIBLUA_GIT_NAME "lua-5.1.5_Linux_armv7l_gcc4.8" CACHE STRING "lua git repo name")
    set(RBK_LIBPROTOBUF_GIT_NAME "protobuf-3.3.0_Linux_armv7l_gcc4.8" CACHE STRING "protobuf git repo name")
    set(RBK_LIBBFL_GIT_NAME "orocos_bfl-0.8.99_Linux_armv7l_gcc4.8" CACHE STRING "bfl git repo name")
    set(RBK_LIBURG_GIT_NAME "urg-1.2.0_Linux_armv7l_gcc4.8" CACHE STRING "urg git repo name")
    set(RBK_LIBSQLITE3_GIT_NAME "sqlite-3.17.0_Linux_armv7l_gcc4.8" CACHE STRING "sqlite3 git repo name")
ELSEIF(WIN32)
    # windows
    IF(CMAKE_CL_64)
        # x64
        set(RBK_LIBLUA_GIT_NAME "lua-5.1.5_Win64_msvc14" CACHE STRING "lua git repo name")
        set(RBK_LIBPROTOBUF_GIT_NAME "protobuf-3.3.0_Win64_msvc14" CACHE STRING "protobuf git repo name")
        set(RBK_LIBBFL_GIT_NAME "orocos_bfl-0.8.99_Win64_msvc14" CACHE STRING "bfl git repo name")
        set(RBK_LIBURG_GIT_NAME "urg-1.2.0_Win64_msvc14" CACHE STRING "urg git repo name")
        set(RBK_LIBARIA_GIT_NAME "Aria-2.9.1-1_Win64_msvc14" CACHE STRING "aria git repo name")
        set(RBK_LIBSQLITE3_GIT_NAME "sqlite-3.17.0_Win64" CACHE STRING "sqlite3 git repo name")

    ELSE(CMAKE_CL_64)
        # x86
        # Visual Studio 2015
        set(RBK_LIBLUA_GIT_NAME "lua-5.1.5_Win32_msvc14" CACHE STRING "lua git repo name")
        set(RBK_LIBPROTOBUF_GIT_NAME "protobuf-3.3.0_Win32_msvc14" CACHE STRING "protobuf git repo name")
        set(RBK_LIBBFL_GIT_NAME "orocos_bfl-0.8.99_Win32_msvc14" CACHE STRING "bfl git repo name")
        set(RBK_LIBURG_GIT_NAME "urg-1.2.0_Win32_msvc14" CACHE STRING "urg git repo name")
        set(RBK_LIBARIA_GIT_NAME "Aria-2.9.1-1_Win32_msvc14" CACHE STRING "aria git repo name")
        set(RBK_LIBSQLITE3_GIT_NAME "sqlite-3.17.0_Win32" CACHE STRING "sqlite3 git repo name")
    ENDIF()

ELSEIF(APPLE)
    # macOS

ELSEIF(UNIX)
    # linux
    set(RBK_LIBLUA_GIT_NAME "lua-5.1.5_Linux_x64_gcc4.8" CACHE STRING "lua git repo name")
    set(RBK_LIBPROTOBUF_GIT_NAME "protobuf-3.3.0_Linux_x64_gcc4.8" CACHE STRING "protobuf git repo name")
    set(RBK_LIBBFL_GIT_NAME "orocos_bfl-0.8.99_Linux_x64_gcc4.8" CACHE STRING "bfl git repo name")
    set(RBK_LIBURG_GIT_NAME "urg-1.2.0_Linux_x64_gcc4.8" CACHE STRING "urg git repo name")
    set(RBK_LIBARIA_GIT_NAME "Aria-2.9.1_Linux_x64_gcc4.8" CACHE STRING "aria git repo name")
    set(RBK_LIBSQLITE3_GIT_NAME "sqlite-3.17.0_Linux_x64_gcc4.8" CACHE STRING "sqlite3 git repo name")
ENDIF()

message("RBK_BOOST_DIR: " ${RBK_BOOST_DIR})
message("RBK_QT_DIR: " ${RBK_QT_DIR})
message("RBK_THIRDPARTY_LIB_DIR: " ${RBK_THIRDPARTY_LIB_DIR})

IF (NOT IS_SUBPROJECT)
    # gitlab repo ip address
    set(RBK_GIT_IP "192.168.2.109" CACHE STRING "git ip address")
    mark_as_advanced(RBK_GIT_IP)

    # gitlib repo owner
    set(RBK_GIT_USER seer-robotics-public CACHE STRING "git repo owner")
    mark_as_advanced(RBK_GIT_USER)

    message("RBK_GIT_IP: " ${RBK_GIT_IP})
    message("RBK_GIT_USER: " ${RBK_GIT_USER})
ENDIF()

message("")

# only use git to download third party libraries when build RoboKit
IF (NOT IS_SUBPROJECT)
    find_package(Git)
    IF(Git_FOUND)
        message("GIT_EXECUTABLE: " ${GIT_EXECUTABLE})
        message("")

        set(LIBLUA_GIT_PATH ${RBK_GIT_USER}/${RBK_LIBLUA_GIT_NAME}.git)
        set(LIBPROTOBUF_GIT_PATH ${RBK_GIT_USER}/${RBK_LIBPROTOBUF_GIT_NAME}.git)
        set(LIBBFL_GIT_PATH ${RBK_GIT_USER}/${RBK_LIBBFL_GIT_NAME}.git)
        set(LIBURG_GIT_PATH ${RBK_GIT_USER}/${RBK_LIBURG_GIT_NAME}.git)
        set(LIBARIA_GIT_PATH ${RBK_GIT_USER}/${RBK_LIBARIA_GIT_NAME}.git)
        set(LIBSQLITE3_GIT_PATH ${RBK_GIT_USER}/${RBK_LIBSQLITE3_GIT_NAME}.git)

        IF(WIN32)
            set(LIBLUA_GIT_URL https://${RBK_GIT_IP}/${LIBLUA_GIT_PATH})
            set(LIBPROTOBUF_GIT_URL https://${RBK_GIT_IP}/${LIBPROTOBUF_GIT_PATH})
            set(LIBBFL_GIT_URL https://${RBK_GIT_IP}/${LIBBFL_GIT_PATH})
            set(LIBURG_GIT_URL https://${RBK_GIT_IP}/${LIBURG_GIT_PATH})
            set(LIBARIA_GIT_URL https://${RBK_GIT_IP}/${LIBARIA_GIT_PATH})
            set(LIBSQLITE3_GIT_URL https://${RBK_GIT_IP}/${LIBSQLITE3_GIT_PATH})
        ELSE()
            set(LIBLUA_GIT_URL git@${RBK_GIT_IP}:${LIBLUA_GIT_PATH})
            set(LIBPROTOBUF_GIT_URL git@${RBK_GIT_IP}:${LIBPROTOBUF_GIT_PATH})
            set(LIBBFL_GIT_URL git@${RBK_GIT_IP}:${LIBBFL_GIT_PATH})
            set(LIBURG_GIT_URL git@${RBK_GIT_IP}:${LIBURG_GIT_PATH})
            set(LIBARIA_GIT_URL git@${RBK_GIT_IP}:${LIBARIA_GIT_PATH})
            set(LIBSQLITE3_GIT_URL git@${RBK_GIT_IP}:${LIBSQLITE3_GIT_PATH})
        ENDIF()
    ENDIF()
ENDIF()

set(LIBLUA_DIR ${RBK_THIRDPARTY_LIB_DIR}/${RBK_LIBLUA_GIT_NAME})
set(LIBPROTOBUF_DIR ${RBK_THIRDPARTY_LIB_DIR}/${RBK_LIBPROTOBUF_GIT_NAME})
set(LIBBFL_DIR ${RBK_THIRDPARTY_LIB_DIR}/${RBK_LIBBFL_GIT_NAME})
set(LIBURG_DIR ${RBK_THIRDPARTY_LIB_DIR}/${RBK_LIBURG_GIT_NAME})
set(LIBARIA_DIR ${RBK_THIRDPARTY_LIB_DIR}/${RBK_LIBARIA_GIT_NAME})
set(LIBSQLITE3_DIR ${RBK_THIRDPARTY_LIB_DIR}/${RBK_LIBSQLITE3_GIT_NAME})

mark_as_advanced(RBK_LIBLUA_GIT_NAME)
mark_as_advanced(RBK_LIBPROTOBUF_GIT_NAME)
mark_as_advanced(RBK_LIBBFL_GIT_NAME)
mark_as_advanced(RBK_LIBURG_GIT_NAME)
mark_as_advanced(RBK_LIBARIA_GIT_NAME)
mark_as_advanced(RBK_LIBSQLITE3_GIT_NAME)
