# Make compilation not verbose by default
# ---------------------------------------
set(CMAKE_VERBOSE_MAKEFILE FALSE)
set(CMAKE_COLOR_MAKEFILE ON)

# project dirs
# ------------
set(PROJ_SOURCE_DIR ${${PROJECT_NAME}_SOURCE_DIR})
set(PROJ_BINARY_DIR ${${PROJECT_NAME}_BINARY_DIR})

# turn on FOLDERS on IDE, such as Visual Studio
# ---------------------------------------------
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# some path
# ---------
IF(IS_SUBPROJECT)
    list(APPEND CMAKE_MODULE_PATH ${RBK_PATH})
    list(APPEND CMAKE_MODULE_PATH ${RBK_PATH}/cmake)
ENDIF()
list(APPEND CMAKE_INCLUDE_PATH ${RBK_THIRDPARTY_LIB_DIR})
list(APPEND CMAKE_LIBRARY_PATH ${RBK_THIRDPARTY_LIB_DIR})

set(RBK_SHARED_LIB "RBK_SHARED_LIB")
set(RBK_STATIC_LIB "RBK_STATIC_LIB")
