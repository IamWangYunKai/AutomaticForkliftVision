IF(WIN32)
    # windows

    add_definitions(/W3 /wd4996) #/wd4995
    add_definitions(-DUNICODE -D_UNICODE) # UNICODE for Windows headers, _UNICODE for C-runtime
    add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS) # ignore warning C4996
    add_definitions(-DNOMINMAX)
    add_definitions(-D_REENTRANT)
    add_definitions(-DBOOST_ALL_DYN_LINK)

    # Release
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi /GL /Gy /GR /GS /Gm- /Gd /GF /O2 /Ob2 /Oi /MD /MP /TP /fp:precise" CACHE STRING "c++ release flags" FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG /LTCG /INCREMENTAL:NO /OPT:REF /OPT:ICF" CACHE STRING "shared linker release flags" FORCE)
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} /LTCG" CACHE STRING "static linker release flags" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG /LTCG /INCREMENTAL:NO /OPT:REF /OPT:ICF" CACHE STRING "exe linker release flags" FORCE)

    # Debug
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /ZI /GR /GS /Gm /Gd /Od /Ob0 /MDd /TP /fp:precise" CACHE STRING "c++ debug flags" FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DEBUG /SAFESEH:NO /INCREMENTAL /OPT:NOREF /OPT:NOICF" CACHE STRING "shared linker debug flags" FORCE)
    set(CMAKE_STATIC_LINKER_FLAGS_DEBUG "${CMAKE_STATIC_LINKER_FLAGS_DEBUG}" CACHE STRING "static linker debug flags" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG /SAFESEH:NO /INCREMENTAL /OPT:NOREF /OPT:NOICF" CACHE STRING "exe linker debug flags" FORCE)

    # RelWithDebInfo
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /ZI /GL /Gy /GR /GS /Gm- /Gd /GF /O2 /Ob1 /Oi /MD /MP /TP /fp:precise" CACHE STRING "c++ relwithdebinfo flags" FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} /DEBUG /LTCG /INCREMENTAL:NO /OPT:REF /OPT:ICF" CACHE STRING "shared linker relwithdebinfo flags" FORCE)
    set(CMAKE_STATIC_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_STATIC_LINKER_FLAGS_RELWITHDEBINFO} /LTCG" CACHE STRING "static linker relwithdebinfo flags" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} /DEBUG /LTCG /INCREMENTAL:NO /OPT:REF /OPT:ICF" CACHE STRING "exe linker relwithdebinfo flags" FORCE)

ELSEIF(UNIX)
    # macOS and linux

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -ftls-model=global-dynamic -pthread -D_REENTRANT")

    # Release
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "c++ release flags" FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}" CACHE STRING "shared linker release flags" FORCE)
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE}" CACHE STRING "static linker release flags" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "exe linker release flags" FORCE)

    # Debug
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}" CACHE STRING "c++ debug flags" FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG}" CACHE STRING "shared linker debug flags" FORCE)
    set(CMAKE_STATIC_LINKER_FLAGS_DEBUG "${CMAKE_STATIC_LINKER_FLAGS_DEBUG}" CACHE STRING "static linker debug flags" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG}" CACHE STRING "exe linker debug flags" FORCE)

    # RelWithDebInfo
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}" CACHE STRING "c++ relwithdebinfo flags" FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO}" CACHE STRING "shared linker relwithdebinfo flags" FORCE)
    set(CMAKE_STATIC_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO}" CACHE STRING "static linker relwithdebinfo flags" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO}" CACHE STRING "exe linker relwithdebinfo flags" FORCE)

    IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
        IF(NOT APPLE)
            add_definitions(-DDEBUG -Wall -Wcast-align)  # todo: should we use CMAKE_C_FLAGS for these?
        ELSE()
            add_definitions(-DDEBUG -Wall)
        ENDIF()
    ELSEIF(CMAKE_BUILD_TYPE STREQUAL "Release")
        IF(NOT APPLE)
            add_definitions(-DNDEBUG -Wall -Wcast-align)  # todo: should we use CMAKE_C_FLAGS for these?
        ELSE()
            add_definitions(-DNDEBUG -Wall)
        ENDIF()
    ELSEIF(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
        IF(NOT APPLE)
            add_definitions(-DNDEBUG -Wall -Wcast-align)  # todo: should we use CMAKE_C_FLAGS for these?
        ELSE()
            add_definitions(-DNDEBUG -Wall)
        ENDIF()
    ENDIF()

ENDIF()

# macOS options
# ------------
IF(APPLE)
    set(CMAKE_MACOSX_RPATH ON) # explicit turn on MACOSX_RPATH
ENDIF()


# position independent code
# -------------------------
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE) ## -fPIC -fPIE

# set rpath
# ---------
# rpath only works on macOS and linux
IF(UNIX)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    IF(APPLE)
        list(APPEND CMAKE_INSTALL_RPATH "@loader_path") # @loader_path only works on macOS
        list(APPEND CMAKE_INSTALL_RPATH "@loader_path/..")
        list(APPEND CMAKE_INSTALL_RPATH "@loader_path/lib")
        list(APPEND CMAKE_INSTALL_RPATH "@loader_path/../lib")
    ELSE(APPLE)
        list(APPEND CMAKE_INSTALL_RPATH "\$ORIGIN") # $ORIGIN only works on linux
        list(APPEND CMAKE_INSTALL_RPATH "\$ORIGIN/..")
        list(APPEND CMAKE_INSTALL_RPATH "\$ORIGIN/lib")
        list(APPEND CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")
    ENDIF()
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
ENDIF()
