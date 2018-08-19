message("")

IF((RBK_PLUGIN OR RBK_ALGORITHM) AND (NOT IS_SUBPROJECT)) # build indenpendent
    include_directories(${RBK_PATH}/thirdparty_libs)
    include_directories(${RBK_PATH}/include)
    include_directories(${RBK_PATH}/include/robokit/foundation)

    link_directories(${RBK_PATH}/lib)

    message("Includes Dir: " ${RBK_PATH}/thirdparty_libs)
    message("Includes Dir: " ${RBK_PATH}/include)
    message("Includes Dir: " ${RBK_PATH}/include/robokit/foundation)
    message("Link Dir: " ${RBK_PATH}/lib)

ELSE()
    include_directories(${PROJ_SOURCE_DIR}/thirdparty_libs)
    include_directories(${PROJ_SOURCE_DIR}/src)
    include_directories(${PROJ_SOURCE_DIR}/src/robokit/foundation)
    include_dirs("${PROJ_SOURCE_DIR}/src/robokit/algorithm" "src")

    link_directories(${PROJ_SOURCE_DIR}/lib)

    message("Includes Dir: " ${PROJ_SOURCE_DIR}/thirdparty_libs)
    message("Includes Dir: " ${PROJ_SOURCE_DIR}/src)
    message("Includes Dir: " ${PROJ_SOURCE_DIR}/src/robokit/foundation)
    message("Link Dir: " ${PROJ_SOURCE_DIR}/lib)

ENDIF()

include_directories(${Protobuf_INCLUDE_DIR})
include_directories(${LUA_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${SQLite3_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
