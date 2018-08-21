 # urg 1.2.0

find_path(Urg_C_INCLUDE_DIR
    urg_utils.h
    PATHS ${LIBURG_DIR}/include/c ${LIBURG_DIR}/include/urg_c
    NO_DEFAULT_PATH)
mark_as_advanced(Urg_C_INCLUDE_DIR)

find_library(Urg_C_LIBRARY_RELEASE
    NAMES urg urg_c
    PATHS ${LIBURG_DIR}/lib/Release ${LIBURG_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(Urg_C_LIBRARY_RELEASE)

find_library(Urg_C_LIBRARY_DEBUG
    NAMES urg
    PATHS ${LIBURG_DIR}/lib/Debug
    NO_DEFAULT_PATH)
mark_as_advanced(Urg_C_LIBRARY_DEBUG)

find_path(Urg_CPP_INCLUDE_DIR
    Urg_driver.h
    PATHS ${LIBURG_DIR}/include/cpp ${LIBURG_DIR}/include/urg_cpp
    NO_DEFAULT_PATH)
mark_as_advanced(Urg_CPP_INCLUDE_DIR)

find_library(Urg_CPP_LIBRARY_RELEASE
    NAMES urg_cpp
    PATHS ${LIBURG_DIR}/lib/Release ${LIBURG_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(Urg_CPP_LIBRARY_RELEASE)

find_library(Urg_CPP_LIBRARY_DEBUG
    NAMES urg_cpp
    PATHS ${LIBURG_DIR}/lib/Debug
    NO_DEFAULT_PATH)
mark_as_advanced(Urg_CPP_LIBRARY_DEBUG)

select_library_configurations(Urg_C)
select_library_configurations(Urg_CPP)

set(Urg_VERSION "1.2.0")

message("")
find_package_handle_standard_args(Urg_C
                                  REQUIRED_VARS Urg_C_LIBRARY Urg_C_INCLUDE_DIR
                                  VERSION_VAR Urg_VERSION)
find_package_handle_standard_args(Urg_CPP
                                  REQUIRED_VARS Urg_CPP_LIBRARY Urg_CPP_INCLUDE_DIR
                                  VERSION_VAR Urg_VERSION)

message("Urg_VERSION: " ${Urg_VERSION})
message("Urg_C_INCLUDE_DIR: " ${Urg_C_INCLUDE_DIR})
message("Urg_CPP_INCLUDE_DIR: " ${Urg_CPP_INCLUDE_DIR})
message("Urg_C_LIBRARY: " ${Urg_C_LIBRARY})
message("Urg_CPP_LIBRARY: " ${Urg_CPP_LIBRARY})
message("")
