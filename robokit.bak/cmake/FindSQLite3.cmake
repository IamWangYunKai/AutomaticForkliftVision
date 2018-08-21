# - Find sqlite3
# Find the native SQLITE3 headers and libraries.
#
# SQLITE3_INCLUDE_DIRS	- where to find sqlite3.h, etc.
# SQLITE3_LIBRARIES	- List of libraries when using sqlite.
# SQLITE3_FOUND	- True if sqlite found.

# Look for the header file.
find_path(SQLite3_INCLUDE_DIR
    NAMES sqlite3.h
    PATHS ${LIBSQLITE3_DIR}/include
    NO_DEFAULT_PATH)
mark_as_advanced(SQLite3_INCLUDE_DIR)

IF(WIN32)

# Look for the library.
find_library(SQLite3_LIBRARY_RELEASE
    NAMES sqlite3
    PATHS ${LIBSQLITE3_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(SQLite3_LIBRARY_RELEASE)

set(TEMP_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})
set(CMAKE_FIND_LIBRARY_SUFFIXES ".dll")

find_library(SQLite3_DLL_RELEASE
    NAMES sqlite3
    PATHS ${LIBSQLITE3_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(SQLite3_DLL_RELEASE)

set(CMAKE_FIND_LIBRARY_SUFFIXES ${TEMP_SUFFIXES})
unset(TEMP_SUFFIXES)

ELSE()

find_library(SQLite3_LIBRARY_RELEASE
    NAMES sqlite3
    PATHS ${LIBSQLITE3_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(SQLite3_LIBRARY_RELEASE)

ENDIF()

select_library_configurations(SQLite3)

set(SQLite3_VERSION "3.17.0")

message("")

FIND_PACKAGE_HANDLE_STANDARD_ARGS(SQLite3
                                  REQUIRED_VARS SQLite3_LIBRARY SQLite3_INCLUDE_DIR
                                  VERSION_VAR SQLite3_VERSION)

# Copy the results to the output variables.
IF(SQLite3_FOUND)
	SET(SQLITE3_LIBRARIES ${SQLite3_LIBRARY})
	SET(SQLITE3_INCLUDE_DIRS ${SQLite3_INCLUDE_DIR})
ELSE(SQLite3_FOUND)
	SET(SQLITE3_LIBRARIES)
	SET(SQLITE3_INCLUDE_DIRS)
ENDIF(SQLite3_FOUND)

message("SQLite3_VERSION: " ${SQLite3_VERSION})
message("SQLite3_INCLUDE_DIR: " ${SQLite3_INCLUDE_DIR})
message("SQLite3_LIBRARY: " ${SQLite3_LIBRARY})

IF(WIN32)
message("SQLite3_DLL_RELEASE: " ${SQLite3_DLL_RELEASE})
ENDIF()

message("")
