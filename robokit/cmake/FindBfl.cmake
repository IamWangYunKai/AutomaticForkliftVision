 # bfl 0.8.99

find_path(Bfl_INCLUDE_DIR
    bfl
    PATHS ${LIBBFL_DIR}/include
    NO_DEFAULT_PATH)
mark_as_advanced(Bfl_INCLUDE_DIR)

find_library(Bfl_LIBRARY_RELEASE
    NAMES orocos-bfl libbfl
    PATHS ${LIBBFL_DIR}/lib/Release ${LIBBFL_DIR}/lib
    NO_DEFAULT_PATH)
mark_as_advanced(Bfl_LIBRARY_RELEASE)

find_library(Bfl_LIBRARY_DEBUG
    NAMES orocos-bfl
    PATHS ${LIBBFL_DIR}/lib/Debug
    NO_DEFAULT_PATH)
mark_as_advanced(Bfl_LIBRARY_DEBUG)

select_library_configurations(Bfl)

set(Bfl_VERSION "0.8.99")

message("")
find_package_handle_standard_args(Bfl
                                  REQUIRED_VARS Bfl_LIBRARY Bfl_INCLUDE_DIR
                                  VERSION_VAR Bfl_VERSION)

message("Bfl_VERSION: " ${Bfl_VERSION})
message("Bfl_INCLUDE_DIR: " ${Bfl_INCLUDE_DIR})
message("Bfl_LIBRARY_RELEASE: " ${Bfl_LIBRARY_RELEASE})
message("Bfl_LIBRARY_DEBUG: " ${Bfl_LIBRARY_DEBUG})
message("Bfl_LIBRARY: " ${Bfl_LIBRARY})
message("")
