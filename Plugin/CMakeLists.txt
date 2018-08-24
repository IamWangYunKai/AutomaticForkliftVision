cmake_minimum_required(VERSION 3.5.2)

# WIN32 means windows, including Win64
# APPLE means macOS
# UNIX means Unix like system

project(IFMO3d303)

set(RBK_PLUGIN "RBK_PLUGIN")

set(RBK_PATH "" CACHE PATH "RoboKit SDK path")
IF(RBK_PATH STREQUAL "")
	message(FATAL_ERROR "RBK_PATH is not set")
ENDIF()

list(APPEND CMAKE_MODULE_PATH ${RBK_PATH}/cmake)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(GetSubprojectStatus)
# is sub project
get_subproject_status(IS_SUBPROJECT)

# configs when build as independent project
# -----------------------------------------
IF(NOT IS_SUBPROJECT)
	# config file
	# -----------
	include(rbkConfig)
	# global settings
	# ---------------
	include(GlobalSettings)
	# Include our cmake macros
	# ------------------------
	include(CMakeHelpers)
	# logging
	# -------
	include(CMakeLogging)
	# build type (affects debugging symbols and optimization)
	# -------------------------------------------------------
	include(BuildType)
	# compiler flags
	# --------------
	include(CompilerFlags)
	# find libs
	# ---------
	message("Finding libraries:")
	message("-----------------------------------------------------")
	include(GetThirdPartyLibs)
	# default includes
	# ----------------
	include(DefaultIncludes)
	# other settings
	# --------------
	include(RBKOtherSettings)
ENDIF()

# custom config file
# ------------------
include(rbkPluginConfig)
message("")

# add subfolders
# --------------
###################################################################
# output directory
# ----------------
include(SetOutputDirectory)
# find_package( PCL 1.8.0 REQUIRED )
# pre process
# -----------
IF(WIN32)
	protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${RBK_PROTO_DIR}/message_localization.proto
											${RBK_PROTO_DIR}/message_header.proto
                                            ${RBK_PROTO_DIR}/message_pointcloud.proto)
ENDIF()

# define
# ------
add_definitions(-D__RBK_PLUGIN__="${PROJECT_NAME}"
                # ${PCL_DEFINITIONS}
                )

# source
# ------
# aux_source_directory("${CMAKE_CURRENT_LIST_DIR}/src/*.*" SRCS)
file(GLOB SRCS "${CMAKE_CURRENT_LIST_DIR}/src/*.*")

# include
# -------
IF(UNIX)
	IF(NOT IS_SUBPROJECT)
		include_directories(BEFORE ${RBK_PATH}/include/robokit/protocol)
	ELSE()
		include_directories(BEFORE ${CMAKE_SOURCE_DIR}/src/robokit/protocol)
	ENDIF()
ENDIF()


# link
# ----

# for VS solution file
# --------------------
# IF(WIN32)
#     file(GLOB HEADERS "*.h")
#     source_group("include" FILES ${HEADERS})
#     source_group("src" FILES ${SRCS})
# ENDIF()

# target
# ------
IF(WIN32)
    # windows
    include(generate_product_version)
	generate_product_version(
        VersionFilesOutputVariable
        NAME "${PROJECT_NAME}"
        ICON ${RBK_PATH}/robokit.ico
        VERSION_MAJOR ${${PROJ_NAME}_VERSION_MAJOR}
        VERSION_MINOR ${${PROJ_NAME}_VERSION_MINOR}
        VERSION_PATCH ${${PROJ_NAME}_VERSION_PATCH}
        COMPANY_NAME ${ROBOTKIT_COMPANY_NAME}
        ORIGINAL_FILENAME "[${ROBOTKIT_NAME}] ${PROJECT_NAME}"
        INTERNAL_NAME "[${ROBOTKIT_NAME}] ${PROJECT_NAME}"
        FILE_DESCRIPTION "[${ROBOTKIT_NAME}] ${PROJECT_NAME}"
    )
	# 头文件
    file(GLOB HEADERS  ${CMAKE_CURRENT_LIST_DIR}/include/third_party/pmdsdk/include
                       ${CMAKE_CURRENT_LIST_DIR}/include/third_party/xmlrpc/include
                       ${CMAKE_CURRENT_LIST_DIR}/include/third_party/xmlrpc/lib/expat/xmlparse
                       ${CMAKE_CURRENT_LIST_DIR}/include/third_party/xmlrpc/lib/util/include
                       ${CMAKE_CURRENT_LIST_DIR}/include/third_party/xmlrpc/lib
                       ${CMAKE_CURRENT_LIST_DIR}/include/third_party/xmlrpc/examples
                       ${CMAKE_CURRENT_LIST_DIR}/include/third_party/xmlrpc
                       ${CMAKE_CURRENT_LIST_DIR}/include/source_plugin/include
                       ${CMAKE_CURRENT_LIST_DIR}/include/third_party/pcl/include/pcl-1.8
                       ${CMAKE_CURRENT_LIST_DIR}/include/third_party/flann/include
                       ${CMAKE_CURRENT_LIST_DIR}/include/third_party/vtk/include/vtk-8.0

                )
	# 库文件
	file(GLOB LIB_DIRS 
                    # ${PCL_LIBRARY_DIRS}
                    ${CMAKE_CURRENT_LIST_DIR}/include/third_party/pcl/lib
                    ${CMAKE_CURRENT_LIST_DIR}/include/third_party/flann/lib
                    ${CMAKE_CURRENT_LIST_DIR}/include/third_party/vtk/lib
                    )
    file(GLOB DLL_DIRS  "${CMAKE_CURRENT_LIST_DIR}/include/third_party/pmdsdk/bin/pmdaccess2.dll"
                        "${CMAKE_CURRENT_LIST_DIR}/include/O3D303_plugins/O3D3xxCamera.W32.pap"
                        "${CMAKE_CURRENT_LIST_DIR}/include/O3D303_plugins/O3D3xxProc.W32.ppp")
    

	include_directories(${HEADERS})
	link_directories(${LIB_DIRS})
    add_library(
			${PROJECT_NAME} SHARED
            ${SRCS}
            ${HEADERS}
            ${PROTO_SRCS}
            ${PROTO_HDRS}
            ${VersionFilesOutputVariable}
			)
    target_link_libraries(
            ${PROJECT_NAME}
            # ${PCL_LIBRARIES}
            core
            foundation
            ray_trace
			chasis
            ${LUA_LIBRARIES}
			${Protobuf_LIBRARIES}
            ${SQLite3_LIBRARY}
            ${CMAKE_CURRENT_LIST_DIR}/include/third_party/pmdsdk/lib/pmdaccess2.lib
            ${CMAKE_CURRENT_LIST_DIR}/include/third_party/pcl/lib/*.lib
            ${CMAKE_CURRENT_LIST_DIR}/include/third_party/flann/lib/*.lib
            ${CMAKE_CURRENT_LIST_DIR}/include/third_party/vtk/lib/*.lib
            AdvAPI32.Lib
            ComDlg32.Lib
            Gdi32.Lib
            Kernel32.Lib
            Ole32.Lib
            OleAut32.Lib
            shell32.lib
            User32.Lib
            Uuid.Lib
            WinSpool.Lib
    )
ENDIF()

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "rbk_plugin")