# Disable unnecessary build types
# -------------------------------
IF(MSVC10)
    set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "Configurations" FORCE)
ELSE()
    set(CMAKE_CONFIGURATION_TYPES "Release;RelWithDebInfo;Debug" CACHE STRING "Configurations" FORCE)
ENDIF()

message("Build type:")
message("-----------------------------------------------------")
IF (NOT CMAKE_BUILD_TYPE)
    message("CMAKE_BUILD_TYPE is DEBUG")
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type: None, Release, Debug, RelWithDebInfo, MinSizeRel." FORCE)
ENDIF()
message("Build type set to ${CMAKE_BUILD_TYPE}")

IF(WIN32)
    IF(CMAKE_CL_64)
        message("Build platform set to Win64")
    ELSE(CMAKE_CL_64)
        message("Build platform set to Win32")
    ENDIF()
ENDIF()

option(RBK_ENABLE_CPP11 "Build with C++11 features enabled." ON)
message("RBK_ENABLE_CPP11: " ${RBK_ENABLE_CPP11})
