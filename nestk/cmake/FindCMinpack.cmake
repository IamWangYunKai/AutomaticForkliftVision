###############################################################################
# Find CMinpack
#
# This sets the following variables:
# CMINPACK_FOUND - True if CMinpack was found.
# CMINPACK_INCLUDE_DIRS - Directories containing the CMinpack include files.
# CMINPACK_LIBRARIES - Libraries needed to use CMinpack.
# CMINPACK_LIBRARIES_DEBUG - Libraries needed to use CMinpack, debug version.
# CMINPACK_DEFINITIONS - Compiler flags for CMinpack.

find_package(PkgConfig)
pkg_check_modules(PC_CMINPACK cminpack)
set(CMINPACK_DEFINITIONS ${PC_CMINPACK_CFLAGS_OTHER})

find_path(CMINPACK_INCLUDE_DIR cminpack.h
          HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_CMINPACK_INCLUDEDIR} ${PC_CMINPACK_INCLUDE_DIRS} "${CMINPACK_ROOT}" "$ENV{CMINPACK_ROOT}"
          PATHS "$ENV{PROGRAMFILES}/CMINPACK 1.1.3" "$ENV{PROGRAMW6432}/CMINPACK 1.1.3" 
          PATH_SUFFIXES include/cminpack-1 include/cminpack include)

# Prefer static libraries in Windows over shared ones
if(WIN32)
  find_library(CMINPACK_LIBRARY 
               NAMES cminpack_s cminpack
               HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_CMINPACK_LIBDIR} ${PC_CMINPACK_LIBRARY_DIRS} "${CMINPACK_ROOT}" "$ENV{CMINPACK_ROOT}"
               PATHS "$ENV{PROGRAMFILES}/CMINPACK 1.1.3" "$ENV{PROGRAMW6432}/CMINPACK 1.1.3" 
               PATH_SUFFIXES lib lib64)

  find_library(CMINPACK_LIBRARY_DEBUG 
               NAMES cminpack_s-gd cminpack-gd cminpack_s cminpack
               HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_CMINPACK_LIBDIR} ${PC_CMINPACK_LIBRARY_DIRS} "${CMINPACK_ROOT}" "$ENV{CMINPACK_ROOT}"
               PATHS "$ENV{PROGRAMFILES}/CMINPACK 1.1.3" "$ENV{PROGRAMW6432}/CMINPACK 1.1.3" 
               PATH_SUFFIXES lib lib64)
else(WIN32)
  find_library(CMINPACK_LIBRARY 
               NAMES cminpack
               HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_CMINPACK_LIBDIR} ${PC_CMINPACK_LIBRARY_DIRS} "${CMINPACK_ROOT}" "$ENV{CMINPACK_ROOT}"
               PATH_SUFFIXES lib)

  find_library(CMINPACK_LIBRARY_DEBUG 
               NAMES cminpack-gd cminpack
               HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_CMINPACK_LIBDIR} ${PC_CMINPACK_LIBRARY_DIRS} "${CMINPACK_ROOT}" "$ENV{CMINPACK_ROOT}"
               PATH_SUFFIXES lib)
endif(WIN32)

if(NOT CMINPACK_LIBRARY_DEBUG)
  set(CMINPACK_LIBRARY_DEBUG ${CMINPACK_LIBRARY})
endif(NOT CMINPACK_LIBRARY_DEBUG)

set(CMINPACK_INCLUDE_DIRS ${CMINPACK_INCLUDE_DIR})
set(CMINPACK_LIBRARIES optimized ${CMINPACK_LIBRARY} debug ${CMINPACK_LIBRARY_DEBUG})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CMinpack DEFAULT_MSG
    CMINPACK_LIBRARY CMINPACK_INCLUDE_DIR)

mark_as_advanced(CMINPACK_LIBRARY CMINPACK_LIBRARY_DEBUG CMINPACK_INCLUDE_DIR)

if(CMINPACK_FOUND)
  message(STATUS "CMinPack found (include: ${CMINPACK_INCLUDE_DIRS}, libs: ${CMINPACK_LIBRARIES})")
  if(WIN32)
    option(CMINPACK_IS_STATIC "Set to OFF if you use shared cminpack library." ON)
    mark_as_advanced(CMINPACK_IS_STATIC)
    if(CMINPACK_IS_STATIC)
      add_definitions(-DCMINPACK_NO_DLL)
    endif(CMINPACK_IS_STATIC)
  endif(WIN32)
endif(CMINPACK_FOUND)
