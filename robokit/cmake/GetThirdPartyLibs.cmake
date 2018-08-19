# lua 5.1.5
IF((NOT EXISTS ${LIBLUA_DIR}) AND (Git_FOUND) AND (NOT IS_SUBPROJECT))
    message("Lua is not exist. Begin to download from ${LIBLUA_GIT_URL}")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} clone ${LIBLUA_GIT_URL}
        WORKING_DIRECTORY ${RBK_THIRDPARTY_LIB_DIR}
        TIMEOUT 120
    )
ENDIF()
include(GetLua)

# protobuf 3.3.0
IF((NOT EXISTS ${LIBPROTOBUF_DIR}) AND (Git_FOUND) AND (NOT IS_SUBPROJECT))
    message("Protobuf is not exist. Begin to download from ${LIBPROTOBUF_GIT_URL}")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} clone ${LIBPROTOBUF_GIT_URL}
        WORKING_DIRECTORY ${RBK_THIRDPARTY_LIB_DIR}
        TIMEOUT 120
    )
ENDIF()
include(GetProtobuf)

# boost >= 1.60.0
include(GetBoost)

# sqlite3 3.17.0
IF((NOT EXISTS ${LIBSQLITE3_DIR}) AND (Git_FOUND) AND (NOT IS_SUBPROJECT))
    message("sqlite3 is not exist. Begin to download from ${LIBSQLITE3_GIT_URL}")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} clone ${LIBSQLITE3_GIT_URL}
        WORKING_DIRECTORY ${RBK_THIRDPARTY_LIB_DIR}
        TIMEOUT 120
    )
ENDIF()
find_package(SQLite3 REQUIRED)

IF(NOT RBK_PLUGIN)

# bfl 0.8.99
IF((NOT EXISTS ${LIBBFL_DIR}) AND (Git_FOUND) AND (NOT IS_SUBPROJECT))
    message("bfl is not exist. Begin to download from ${LIBBFL_GIT_URL}")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} clone ${LIBBFL_GIT_URL}
        WORKING_DIRECTORY ${RBK_THIRDPARTY_LIB_DIR}
        TIMEOUT 120
    )
ENDIF()
find_package(Bfl)

# urg 1.2.0
IF((NOT EXISTS ${LIBURG_DIR}) AND (Git_FOUND) AND (NOT IS_SUBPROJECT))
    message("urg is not exist. Begin to download from ${LIBURG_GIT_URL}")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} clone ${LIBURG_GIT_URL}
        WORKING_DIRECTORY ${RBK_THIRDPARTY_LIB_DIR}
        TIMEOUT 120
    )
ENDIF()
find_package(Urg)

# aria 2.9.1-1
IF (NOT CMAKE_CROSSCOMPILING)
IF((NOT EXISTS ${LIBARIA_DIR}) AND (Git_FOUND) AND (NOT IS_SUBPROJECT))
    message("aria is not exist. Begin to download from ${LIBARIA_GIT_URL}")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} clone ${LIBARIA_GIT_URL}
        WORKING_DIRECTORY ${RBK_THIRDPARTY_LIB_DIR}
        TIMEOUT 120
    )
ENDIF()
find_package(Aria)
ENDIF()

ENDIF()
