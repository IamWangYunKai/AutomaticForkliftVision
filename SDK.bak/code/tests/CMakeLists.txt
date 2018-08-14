# Set the project name
SET(PROJ_NAME "${PROJECT_NAME}Tests")

# Add Boost support if it is found
IF(${Boost_FOUND})
	SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES("${BOOST_LINK_DIRS}" ${GTEST_LIB_DIRS})
ENDIF()

ADD_DEFINITIONS(-D_WINSOCK_DEPRECATED_NO_WARNINGS)

# Set the source and header file lists
SET(CXXSOURCES ${CXXSOURCES}
		${CMAKE_CURRENT_SOURCE_DIR}/src/o3d3xx_camera_tests.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/../source_plugin/src/o3d3xx_camera.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/../source_plugin/src/clientsocket.cpp)
		 
SET(HXXSOURCES 
		${CMAKE_CURRENT_SOURCE_DIR}/../source_plugin/include/o3d3xx_camera.hpp)
		
SET(INCLUDE_DIRS ${INCLUDE_DIRS}
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${CMAKE_CURRENT_SOURCE_DIR}/../source_plugin/include/)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS}
		${GTEST_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJ_NAME}  ${CXXSOURCES} ${HXXSOURCES})

TARGET_LINK_LIBRARIES(${PROJ_NAME} ${XMLRPC_LIBS} wininet ws2_32 gtest gtest_main)

# The following include is needed as there is an issue with VS2012 and GTest
IF (MSVC AND MSVC_VERSION EQUAL 1700)
	ADD_DEFINITIONS(-D_VARIADIC_MAX=10)
ENDIF()

ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy "${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/${PROJ_NAME}.exe" "${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/Test/${PROJ_NAME}.exe ")

#remove the binary from configuration folder 
ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E remove -f
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/${PROJ_NAME}.exe ")			