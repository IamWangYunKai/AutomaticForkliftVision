###################################################################################################
# This is the main CMakeLists.txt file for the O3D3xx plugin.
# Author : Jitesh Butala
# Email : jitesh.butala@ifm.com
###################################################################################################

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9 FATAL_ERROR)

PROJECT(O3D3xx)

# Check if the build folder is a subfolder of the code folder
STRING(FIND "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}" BUILD_SUBFOLDER_OF_CODE)

IF(BUILD_SUBFOLDER_OF_CODE GREATER -1)
	MESSAGE(FATAL_ERROR "Build folder cannot be a subfolder of the code folder. Please select another location.")
ENDIF()

# Set the Version Info
FIND_PACKAGE(Subversion)
SET(VERSION_MAJOR "1")
SET(VERSION_MINOR "9")
IF(Subversion_FOUND)
	GET_FILENAME_COMPONENT(ABS_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
	EXECUTE_PROCESS(COMMAND ${Subversion_SVN_EXECUTABLE} info ${ABS_FOLDER}
					RESULT_VARIABLE INFO_RESULT
					OUTPUT_VARIABLE IGNORED
					ERROR_VARIABLE IGNORED)
	IF(INFO_RESULT EQUAL 0)
		# If SVN copy, take the latest version
		Subversion_WC_INFO(${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_NAME})
		
		FILE(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/doxygen/svn_version.bin" "${${PROJECT_NAME}_WC_REVISION}")
		
		SET(VERSION_BUILD "${${PROJECT_NAME}_WC_REVISION}")
	ELSE()
		IF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/doxygen/svn_version.bin")
			# If not SVN copy, use the version stored in the binary file
			FILE(READ "${CMAKE_CURRENT_SOURCE_DIR}/third_party/doxygen/svn_version.bin" VERSION_BUILD)
		ELSE()
			# If nothing present, then set the version to 999999999
			SET(VERSION_BUILD "999999999")
		ENDIF()
	ENDIF()
ELSE()
	IF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/doxygen/svn_version.bin")
		# If not SVN copy, use the version stored in the binary file
		FILE(READ "${CMAKE_CURRENT_SOURCE_DIR}/third_party/doxygen/svn_version.bin" VERSION_BUILD)
	ELSE()
		# If nothing present, then set the version to 999999999
		SET(VERSION_BUILD "999999999")
	ENDIF()
ENDIF()

# Set the output directories
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(BINARY_OUTPUT_PATH ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# Set the common includes
SET(CXXSOURCES)

SET(HXXSOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/pmdsdk/include/pmddatadescription.h
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/pmdsdk/include/pmdsdk2.h
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/pmdsdk/include/pmdsdk2common.h)
		
SET(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pmdsdk/include)

# Give user option to build unit tests
OPTION(O3D3XX_UNIT_TESTS_BUILD_ALL "Build Unit Tests" OFF)

# Give user option to build sample codes
OPTION(O3D3XX_SAMPLE_CODES_BUILD_ALL "Build Sample codes" ON)

# Give user option to build the documentation
OPTION(O3D3XX_DOXYGEN_DOCUMENTATION_GENERATE "Generate Doxygen Documentation" OFF)

# Which type of documentation to be built?
OPTION(O3D3XX_DOXYGEN_DOCUMENTATION_CHM "Generate CHM documentation" ON)
OPTION(O3D3XX_DOXYGEN_DOCUMENTATION_PDF "Generate PDF documentation" OFF)

# Do we want the Release Package Option?
OPTION(O3D3XX_GENERATE_RELEASE_PACKAGE "Create the project for installing the release" OFF)

# Find doxygen if documentation is selected
IF (${O3D3XX_DOXYGEN_DOCUMENTATION_CHM})
	FIND_PACKAGE(doxygen REQUIRED)
	FIND_PACKAGE(HTMLHelp REQUIRED)
ENDIF()

# Find latex only if required
IF (${O3D3XX_DOXYGEN_DOCUMENTATION_PDF})
	FIND_PACKAGE(latex REQUIRED)
ENDIF()


# Find Boost
#FIND_PACKAGE(Boost REQUIRED)
#SET(Boost_FOUND TRUE)
#SET(Boost_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost/include")
#SET(BOOST_LINK_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost/lib")

# Set the project to export the symbols
ADD_DEFINITIONS(-DDLL_EXPORTS)

# Set all the directories
# Add XML-RPC
ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/third_party/xmlrpc)

# Add Source and Processing plugins
ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/source_plugin)
ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/processing_plugin)

# Add sample codes projects if user has selected to build the sample codes
IF (${O3D3XX_SAMPLE_CODES_BUILD_ALL})
	#Add directory which contains sample code
	ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/samples)
ENDIF()

# Add unit testing frameowrk if user has selected to build unit tests
IF (${O3D3XX_UNIT_TESTS_BUILD_ALL})
	# Add GTest subdirectory so that it can also be built along with the project
	ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/third_party/gtest-1.7.0)

	# Set the GTest library & include directories
	SET(GTEST_LIB_DIRS
			${GTEST_LIBRARY}
			${GTEST_MAIN_LIBRARY})
	
	SET(GTEST_INCLUDE_DIRS
			${gtest_SOURCE_DIR}
			${gtest_SOURCE_DIR}/include)
			
	# Add Test subdirectory so that it can also be built along with the project
	ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/tests)
ENDIF()

# Set the include directories
INCLUDE_DIRECTORIES(${INCLUDE_DIRS} ${INCLUDE_DIRS_COMMON})

IF(${O3D3XX_GENERATE_RELEASE_PACKAGE})
	GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PATH)

	# Create a dummy target
ADD_CUSTOM_TARGET(BUILD_INSTALL ALL
		DEPENDS ${PROJECT_NAME}Camera ${PROJECT_NAME}Proc)
		
	IF(MSVC)
		##for VS2008
		IF(MSVC_VERSION EQUAL 1500)
		##lightVis
		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL  POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				"${CMAKE_CURRENT_SOURCE_DIR}/third_party/MSVC/MSVC90"
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/LightVis"
				)

		ENDIF(MSVC_VERSION EQUAL 1500)
		##for VS2010
		IF(MSVC_VERSION EQUAL 1600)

		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL  POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				"${CMAKE_CURRENT_SOURCE_DIR}/third_party/MSVC/MSVC100"
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/LightVis"
				)
		ENDIF(MSVC_VERSION EQUAL 1600)
	
		##for VS2012
		IF(MSVC_VERSION EQUAL 1700)

		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL  POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				"${CMAKE_CURRENT_SOURCE_DIR}/third_party/MSVC/MSVC110"
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/LightVis"
				)

		ENDIF(MSVC_VERSION EQUAL 1700)
	
		###for VS2013
		IF(MSVC_VERSION EQUAL 1800)
		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL  POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				"${CMAKE_CURRENT_SOURCE_DIR}/third_party/MSVC/MSVC120"
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/lightVis"
				)
				ADD_DEFINITIONS(-D_WINSOCK_DEPRECATED_NO_WARNINGS)
		ENDIF(MSVC_VERSION EQUAL 1800)
ENDIF(MSVC)

	# Define the Program Files folder
	SET(PROGRAM_FILES)
	IF(EXISTS $ENV{ProgramW6432})
		SET(PROGRAM_FILES $ENV{ProgramW6432})
	ELSE(EXISTS $ENV{ProgramW6432})
		SET(PROGRAM_FILES $ENV{ProgramFiles})
	ENDIF(EXISTS $ENV{ProgramW6432})

	# Find the ZIP archiver
	UNSET(ZIP_EXECUTABLE CACHE)
	FIND_PROGRAM(ZIP_EXECUTABLE wzzip PATHS "${PROGRAM_FILES}/WinZip")
	IF(ZIP_EXECUTABLE)
		SET(ZIP_COMMAND "${ZIP_EXECUTABLE}")
		SET(ZIP_OPTIONS -P)
	ENDIF()

	IF(NOT ZIP_EXECUTABLE)
		FIND_PROGRAM(ZIP_EXECUTABLE 7z PATHS "${PROGRAM_FILES}/7-Zip")
		IF(ZIP_EXECUTABLE)
			SET(ZIP_COMMAND "${ZIP_EXECUTABLE}")
			SET(ZIP_OPTIONS a -r -tzip)
		ENDIF()
	ENDIF()

	# Set location for the zip folder to be generated
	SET(ZIP_FILE_FOLDER ${BASE_DIR}/release)

	IF(NOT ZIP_EXECUTABLE)
		MESSAGE(FATAL_ERROR "No Archiver found")
	ELSE()
		
			
		# Create archive Lightvis_O3D3xx
		SET(ZIP_FILE_NAME "Bin_O3D3xx_${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_BUILD}.zip")

		# Copy the lightvis
		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/LightVis" 
				"${BINARY_OUTPUT_PATH}/install_lightvis/LightVis"
			)
			#copy plugins folder
		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				"${BINARY_OUTPUT_PATH}/$<CONFIGURATION>/Plugins" 
				"${BINARY_OUTPUT_PATH}/install_lightvis/Plugins"
			)
			
				ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				"${CMAKE_CURRENT_SOURCE_DIR}/third_party/pmdsdk" 
				"${BINARY_OUTPUT_PATH}/install_lightvis/PMDSDK"
			)
			
					
		# Create the compressed archive
		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL POST_BUILD
			COMMAND "${ZIP_COMMAND}" ${ZIP_OPTIONS} ${ZIP_FILE_NAME} ${BINARY_OUTPUT_PATH}/install_lightvis/*.*
			WORKING_DIRECTORY ${ZIP_FILE_FOLDER})

		# Create archive pmdsdk2_o3d3xx_plugin_VERSION
		SET(ZIP_FILE_NAME "pmdsdk2_o3d3xx_plugin_${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_BUILD}.zip")

		# Copy the code
		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				"${BASE_DIR}/code"
				"${BINARY_OUTPUT_PATH}/install_o3d3xx/code"
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				"${BASE_DIR}/tools/lightvis"
				"${BINARY_OUTPUT_PATH}/install_o3d3xx/tools/lightvis"
			)
			
		# Delete the svn_revision.bin file
		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E remove -f
				"${BINARY_OUTPUT_PATH}/install_o3d3xx/code/third_party/doxygen/svn_version.bin"
			)
		
		# Copy the manual and the presentation
		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy
				"${BASE_DIR}/docs/general/O3D3xxCamera_PMD_SDK_Manual_ifm.pdf"
				"${BINARY_OUTPUT_PATH}/install_o3d3xx/docs/O3D3xxCamera_PMD_SDK_Manual_ifm.pdf"
			COMMAND ${CMAKE_COMMAND} -E copy
				"${BASE_DIR}/docs/general/start.pdf"
				"${BINARY_OUTPUT_PATH}/install_o3d3xx/start.pdf"
			)
			
		# Create the compressed archive
		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL POST_BUILD
			COMMAND "${ZIP_COMMAND}" ${ZIP_OPTIONS} ${ZIP_FILE_NAME} ${BINARY_OUTPUT_PATH}/install_o3d3xx/*.*
			WORKING_DIRECTORY ${ZIP_FILE_FOLDER})
			
			# Delete the lightvis and o3d3xx install folders
		ADD_CUSTOM_COMMAND(TARGET BUILD_INSTALL POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E remove_directory
				"${BINARY_OUTPUT_PATH}/install_lightvis"
			COMMAND ${CMAKE_COMMAND} -E remove_directory
				"${BINARY_OUTPUT_PATH}/install_o3d3xx"
			)
			
	ENDIF()
ENDIF()

