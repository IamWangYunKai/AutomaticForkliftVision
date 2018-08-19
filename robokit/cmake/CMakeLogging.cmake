# ------------------------- Begin Generic CMake Variable Logging ------------------

message("System & CMake infomation:")
message("-----------------------------------------------------")

message("CMAKE_VERSION: " ${CMAKE_VERSION})

# the full path to the cmake executable
message("CMAKE_COMMAND: " ${CMAKE_COMMAND})

message("CMAKE_ROOT: " ${CMAKE_ROOT})

message("CMAKE_ARGC: " ${CMAKE_ARGC})

set(INDEX_ARG 0)
while(INDEX_ARG LESS ${CMAKE_ARGC})
    message("CMAKE_ARGV${INDEX_ARG}: " ${CMAKE_ARGV${INDEX_ARG}})
    math(EXPR INDEX_ARG "${INDEX_ARG} + 1")
endwhile()
unset(INDEX_ARG)

# if you are building in-source, this is the same as CMAKE_SOURCE_DIR, otherwise
# this is the top level directory of your build tree
message("CMAKE_BINARY_DIR: " ${CMAKE_BINARY_DIR})

# this is the directory, from which cmake was started, i.e. the top level source directory
message("CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})

# contains the full path to the top level directory of your build tree
message("PROJECT_BINARY_DIR: " ${PROJECT_BINARY_DIR})

# contains the full path to the root of your project source directory,
# i.e. to the nearest directory where CMakeLists.txt contains the PROJECT() command
message("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})

# tell CMake to search first in directories listed in CMAKE_MODULE_PATH
# when you use FIND_PACKAGE() or INCLUDE()
message("CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

# this is used when searching for include files e.g. using the FIND_PATH() command.
message("CMAKE_INCLUDE_PATH: " ${CMAKE_INCLUDE_PATH})

# this is used when searching for libraries e.g. using the FIND_LIBRARY() command.
message("CMAKE_LIBRARY_PATH: " ${CMAKE_LIBRARY_PATH})

# the complete system name, e.g. "Linux-2.4.22", "FreeBSD-5.4-RELEASE" or "Windows 5.1"
message("CMAKE_SYSTEM: " ${CMAKE_SYSTEM})

# the short system name, e.g. "Linux", "FreeBSD" or "Windows"
message("CMAKE_SYSTEM_NAME: " ${CMAKE_SYSTEM_NAME})

# only the version part of CMAKE_SYSTEM
message("CMAKE_SYSTEM_VERSION: " ${CMAKE_SYSTEM_VERSION})

# Visual Studio Windows Target Platform Version
message("CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION: " ${CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION})

# the processor name (e.g. "Intel(R) Pentium(R) M processor 2.00GHz")
message("CMAKE_SYSTEM_PROCESSOR: " ${CMAKE_SYSTEM_PROCESSOR})

# is TRUE then crossing compiling
message("CMAKE_CROSSCOMPILING: " ${CMAKE_CROSSCOMPILING})

# is TRUE on all UNIX-like OS's, including Apple OS X and CygWin
message("UNIX: " ${UNIX})

# is TRUE on Windows, including CygWin
message("WIN32: " ${WIN32})

# is TRUE on Apple OS X
message("APPLE: " ${APPLE})

# is TRUE when using the MinGW compiler in Windows
message("MINGW: " ${MINGW})

# is TRUE on Windows when using the CygWin version of cmake
message("CYGWIN: " ${CYGWIN})

# Microsoft compiler
message("MSVC: " ${MSVC})
message("MSVC_IDE: " ${MSVC_IDE})
message("MSVC_VERSION: " ${MSVC_VERSION})
message("MSVC10: " ${MSVC10})
message("MSVC11: " ${MSVC11})
message("MSVC12: " ${MSVC12})
message("MSVC14: " ${MSVC14})
message("CMAKE_CL_64: " ${CMAKE_CL_64})

# Apple Xcode
message("XCODE: " ${XCODE})
message("XCODE VERSION: " ${XCODE_VERSION})

# whether to disable generation of installation rules
message("CMAKE_SKIP_INSTALL_RULES: " ${CMAKE_SKIP_INSTALL_RULES})

message("CMAKE_C_COMPILER_ID: " ${CMAKE_C_COMPILER_ID})

message("CMAKE_CXX_COMPILER_ID: " ${CMAKE_CXX_COMPILER_ID})

message("CMAKE_C_COMPILER: " ${CMAKE_C_COMPILER})

message("CMAKE_CXX_COMPILER: " ${CMAKE_CXX_COMPILER})

message("CMAKE_COMPILER_IS_GNUCC: " ${CMAKE_COMPILER_IS_GNUCC})

message("CMAKE_COMPILER_IS_GNUCXX: " ${CMAKE_COMPILER_IS_GNUCXX})

message("CMAKE_C_STANDARD: " ${CMAKE_C_STANDARD})

message("CMAKE_CXX_STANDARD: " ${CMAKE_CXX_STANDARD})

message("CMAKE_GENERATOR_TOOLSET: " ${CMAKE_GENERATOR_TOOLSET})

message("CMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE: " ${CMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE})

message("")
