
# Set the project name
SET(PROJ_NAME "${PROJECT_NAME}Camera")

SET(BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

# Configure the version info file
CONFIGURE_FILE("${BASE_DIR}/third_party/doxygen/version.rc.in" "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)

# Add Boost support if it is found
IF(${Boost_FOUND})
	SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES("${BOOST_LINK_DIRS}")
ENDIF()

ADD_DEFINITIONS(-D_WINSOCK_DEPRECATED_NO_WARNINGS)

# Set the source and header file lists
SET(CXXSOURCES ${CXXSOURCES}
		${CMAKE_CURRENT_SOURCE_DIR}/src/o3d3xx_camera.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/o3d3xxSrc.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/clientsocket.cpp)

SET(HXXSOURCES ${HXXSOURCES}
		${CMAKE_CURRENT_SOURCE_DIR}/include/o3d3xx_camera.hpp
		${CMAKE_CURRENT_SOURCE_DIR}/include/clientsocket.hpp)

SET(INCLUDE_DIRS ${INCLUDE_DIRS}
		${CMAKE_CURRENT_SOURCE_DIR}/include)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_LIBRARY(${PROJ_NAME} SHARED
		${CXXSOURCES} ${HXXSOURCES}
		${CMAKE_CURRENT_BINARY_DIR}/version.rc)

TARGET_LINK_LIBRARIES(${PROJ_NAME} ${XMLRPC_LIBS} wininet ws2_32)
			
SET_TARGET_PROPERTIES(${PROJ_NAME} PROPERTIES
		SUFFIX ".W32.pap")


# If the option is selected to build documentation, then add the appropriate commands	
IF(O3D3XX_DOXYGEN_DOCUMENTATION_GENERATE)
	# Add the post build events to generate Doxygen documentation
	IF(DOXYGEN_FOUND)
		# Variables for doxygen
		SET(INCLUDE_TAGFILE "")
		IF(${O3D3XX_DOXYGEN_DOCUMENTATION_CHM})
			SET(BUILD_HTML "YES")
		ELSE()
			SET(BUILD_HTML "NO")
		ENDIF()
		
		IF(${O3D3XX_DOXYGEN_DOCUMENTATION_PDF})
			SET(BUILD_LATEX "YES")
		ELSE()
			SET(BUILD_LATEX "NO")
		ENDIF()
		
		IF (${BUILD_HTML})
			SET(DOXYGEN_CHM_FILE_SRC_PATH "${CMAKE_CURRENT_BINARY_DIR}/docs/html/${PROJECT_NAME}_Plugin.chm")
			SET(DOXYGEN_CHM_FILE_DST_PATH "${CMAKE_CURRENT_BINARY_DIR}/../docs/${PROJECT_NAME}_Plugin.chm")
		ENDIF()
		
		IF (${BUILD_LATEX})
			SET(DOXYGEN_PDF_FILE_SRC_PATH "${CMAKE_CURRENT_BINARY_DIR}/docs/latex/${PROJECT_NAME}_Plugin.pdf")
			SET(DOXYGEN_PDF_FILE_DST_PATH "${CMAKE_CURRENT_BINARY_DIR}/../docs/${PROJECT_NAME}_Plugin.pdf")
		ENDIF()
		
		# Set the input paths for the doxygen
		SET(INPUT_PATH_LIST "${BASE_DIR}/source_plugin/include")
		SET(INPUT_PATH_LIST ${INPUT_PATH_LIST} "${BASE_DIR}/source_plugin/src")
		SET(INPUT_PATH_LIST ${INPUT_PATH_LIST} "${BASE_DIR}/processing_plugin/include")
		SET(INPUT_PATH_LIST ${INPUT_PATH_LIST} "${BASE_DIR}/processing_plugin/src")
		SET(INPUT_PATH_LIST ${INPUT_PATH_LIST} "${BASE_DIR}/samples/include")
		SET(INPUT_PATH_LIST ${INPUT_PATH_LIST} "${BASE_DIR}/samples/src")
		SET(INPUT_PATH_LIST ${INPUT_PATH_LIST} "${BASE_DIR}/third_party/pmdsdk/pmdsdk2common.h") 
		STRING(REPLACE ";" " " INPUT_PATH_LIST "${INPUT_PATH_LIST}")
		
		# Generate the doxyfile for this module
		CONFIGURE_FILE("${BASE_DIR}/third_party/doxygen/Doxyfile.in" "${CMAKE_CURRENT_BINARY_DIR}/docs/Doxyfile" @ONLY)
		
		# Copy the .tex files for LaTex documentation
		ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different
				"${BASE_DIR}/third_party/doxygen/header.tex"
				"${CMAKE_CURRENT_BINARY_DIR}/docs/header.tex"
			COMMAND ${CMAKE_COMMAND} -E copy_if_different
				"${BASE_DIR}/third_party/doxygen/footer.tex"
				"${CMAKE_CURRENT_BINARY_DIR}/docs/footer.tex"
			)
			
		# Create the HTML & CHM doxygen documentation
		ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME} POST_BUILD
			COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/docs/Doxyfile
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs VERBATIM)
			
		IF (${BUILD_LATEX})
			# Create the PDF and copy the help file to the appropriate folder only if PDF is generated
			ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME} POST_BUILD
				COMMAND ${BASE_DIR}/third_party/doxygen/make.bat "O3D3xx_Plugin"
				WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs/latex VERBATIM
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
				${DOXYGEN_PDF_FILE_SRC_PATH} ${DOXYGEN_PDF_FILE_DST_PATH})
		ENDIF()
		
		IF(${BUILD_HTML})
			# Copy the help (.chm) file to the appropriate folder
			ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
				${DOXYGEN_CHM_FILE_SRC_PATH} ${DOXYGEN_CHM_FILE_DST_PATH})
		ENDIF()
		
	ENDIF()
ENDIF()


# Add the post build event to copy the lightvis and related files to the output directory
ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory
		"${BASE_DIR}/../tools/lightvis"
		"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/LightVis"
	)
# Copy the generated pap to plugins and other folder
		ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/${PROJECT_NAME}Camera.W32.pap"
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/Plugins/${PROJECT_NAME}Camera.W32.pap"
				)
		ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME}  POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/Plugins/${PROJECT_NAME}Camera.W32.pap"
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/LightVis/${PROJECT_NAME}Camera.W32.pap"
				)
		ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME}  POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/Plugins/${PROJECT_NAME}Camera.W32.pap"
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/Sample/${PROJECT_NAME}Camera.W32.pap"
				)	
		ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME}  POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/Plugins/${PROJECT_NAME}Proc.W32.ppp"
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/Sample/${PROJECT_NAME}Proc.W32.ppp"
				)	
		#remove the binary from configuration folder 
		ADD_CUSTOM_COMMAND(TARGET ${PROJ_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E remove -f
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/${PROJECT_NAME}Camera.W32.pap")	