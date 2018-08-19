# Aria 2.9.1

find_path(Aria_INCLUDE_DIR
    ArRobot.h
    PATHS ${LIBARIA_DIR}/include
    NO_DEFAULT_PATH)
mark_as_advanced(Aria_INCLUDE_DIR)

IF(WIN32)

find_library(Aria_LIBRARY_RELEASE
    NAMES AriaVC14 aria
    PATHS ${LIBARIA_DIR}/lib/Release ${LIBARIA_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(Aria_LIBRARY_RELEASE)

find_library(Aria_LIBRARY_DEBUG
    NAMES AriaDebugVC14
    PATHS ${LIBARIA_DIR}/lib/Debug ${LIBARIA_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(Aria_LIBRARY_DEBUG)

set(TEMP_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})
set(CMAKE_FIND_LIBRARY_SUFFIXES ".dll")

find_library(Aria_DLL_RELEASE
    NAMES AriaVC14 AriaVC10
    PATHS ${LIBARIA_DIR}/lib/Release ${LIBARIA_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(Aria_DLL_RELEASE)

find_library(Aria_DLL_DEBUG
    NAMES AriaDebugVC14
    PATHS ${LIBARIA_DIR}/lib/Debug ${LIBARIA_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(Aria_DLL_DEBUG)

set(CMAKE_FIND_LIBRARY_SUFFIXES ${TEMP_SUFFIXES})
unset(TEMP_SUFFIXES)

ELSE()

find_library(Aria_LIBRARY_RELEASE
    NAMES Aria
    PATHS ${LIBARIA_DIR}/lib/Release ${LIBARIA_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(Aria_LIBRARY_RELEASE)

ENDIF()

select_library_configurations(Aria)

set(Aria_VERSION "2.9.1")

find_package_handle_standard_args(Aria
                                  REQUIRED_VARS Aria_LIBRARY Aria_INCLUDE_DIR
                                  VERSION_VAR Aria_VERSION)

message("Aria_VERSION: " ${Aria_VERSION})
message("Aria_INCLUDE_DIR: " ${Aria_INCLUDE_DIR})
message("Aria_LIBRARY: " ${Aria_LIBRARY})

IF(WIN32)
message("Aria_DLL_RELEASE: " ${Aria_DLL_RELEASE})
message("Aria_DLL_DEBUG: " ${Aria_DLL_DEBUG})
ENDIF()

message("")
