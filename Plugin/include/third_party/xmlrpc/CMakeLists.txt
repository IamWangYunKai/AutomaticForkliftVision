

CMAKE_MINIMUM_REQUIRED	(VERSION	2.8.0)	

# PROJECT("xmlrpc_opensrc_library")
SET(ALL_HXXSOURCES "")

#libxmlrpc Library
SET(CXXSOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/src/double.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/parse_datetime.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/parse_value.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/resource.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/trace.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/version.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_array.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_build.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_data.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_datetime.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_decompose.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_expat.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_parse.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_serialize.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_string.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_struct.c
)

SET(HXXSOURCES 
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/base.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/base_int.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/c_util.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/config.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/double.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/parse_datetime.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/parse_value.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/registry.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/system_method.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/string_int.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/util.h
	${CMAKE_CURRENT_SOURCE_DIR}/xmlrpc_config.h
	${CMAKE_CURRENT_SOURCE_DIR}/version.h
	#PARENT_SCOPE
)

SET(INCLUDE_DIRS ${INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmlparse
	${CMAKE_CURRENT_SOURCE_DIR}/lib/util/include
	${CMAKE_CURRENT_SOURCE_DIR}/lib
	${CMAKE_CURRENT_SOURCE_DIR}/examples
	${CMAKE_CURRENT_SOURCE_DIR}
	PARENT_SCOPE
	)

SET(ALL_HXXSOURCES HXXSOURCES)

INCLUDE_DIRECTORIES(
	${INCLUDE_DIRS}
)

SET(SOURCES	${CXXSOURCES}	${HXXSOURCES}	)
	SET(MYLIBRARIES	"")
	
ADD_LIBRARY(libxmlrpc	STATIC	${SOURCES}	)

SET(XMLRPC_LIBS ${XMLRPC_LIBS} libxmlrpc)


#libxmlrpc_client Library
SET(CXXSOURCES	
	${CMAKE_CURRENT_SOURCE_DIR}/lib/wininet_transport/pthreadx_win32.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_client.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_client_global.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/xmlrpc_server_info.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/wininet_transport/xmlrpc_wininet_transport.c
)

SET(HXXSOURCES ${HXXSOURCES}
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/base.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/base_int.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/c_util.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/client.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/client_global.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/config.h
	${CMAKE_CURRENT_SOURCE_DIR}/lib/abyss/src/pthreadx.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/server.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/string_int.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/util.h
	#PARENT_SCOPE
	)
	
SET(INCLUDE_DIRS ${INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/lib
	${CMAKE_CURRENT_SOURCE_DIR}/lib/util/include
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/../
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmlparse
	${CMAKE_CURRENT_SOURCE_DIR}/lib/abyss/src
	${CMAKE_CURRENT_SOURCE_DIR}/lib/wininet_transport)
	
SET(ALL_HXXSOURCES ${ALL_HXXSOURCES} HXXSOURCES)
	
INCLUDE_DIRECTORIES(	
	${INCLUDE_DIRS}
)

SET(SOURCES	${CXXSOURCES}	${HXXSOURCES}	)
SET(MYLIBRARIES	"")
	
ADD_LIBRARY(libxmlrpc_client STATIC	${SOURCES})

SET(XMLRPC_LIBS ${XMLRPC_LIBS} libxmlrpc_client)

#libxmlrpc_util Library
SET(CXXSOURCES	
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/asprintf.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/base64.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/error.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/lock_platform.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/lock_windows.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/make_printable.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/memblock.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/select.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/sleep.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/string_number.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/time.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libutil/utf8.c
	)

SET(HXXSOURCES ${HXXSOURCES}
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/base64_int.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/select_int.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/sleep_int.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/string_int.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/string_number.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/time_int.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc-c/util.h
	#PARENT_SCOPE
	)

SET(INCLUDE_DIRS ${INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/lib/util/include
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmlparse
	${CMAKE_CURRENT_SOURCE_DIR}/lib/wininet_transport
	${CMAKE_CURRENT_SOURCE_DIR}/lib/abyss/src)
	
SET(ALL_HXXSOURCES ${ALL_HXXSOURCES} HXXSOURCES)
	
INCLUDE_DIRECTORIES(	
	${INCLUDE_DIRS}
)

SET(SOURCES	${CXXSOURCES}	${HXXSOURCES}	)
SET(MYLIBRARIES	"")
	
ADD_LIBRARY(libxmlrpc_util STATIC	${SOURCES})

SET(XMLRPC_LIBS ${XMLRPC_LIBS} libxmlrpc_util)

#libxmlrpc_xmlparse Library
SET(CXXSOURCES	
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmlparse/xmlparse.c
	)

SET(HXXSOURCES ${HXXSOURCES}
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmlparse/xmlparse.h
	#PARENT_SCOPE
	)
	
SET(INCLUDE_DIRS ${INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmlwf
	${CMAKE_CURRENT_SOURCE_DIR}/lib/util/include
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR})
	
SET(ALL_HXXSOURCES ${ALL_HXXSOURCES} HXXSOURCES)
	
INCLUDE_DIRECTORIES(	
	${INCLUDE_DIRS}
	)

SET(SOURCES	${CXXSOURCES}	${HXXSOURCES}	)
SET(MYLIBRARIES	"")
	
ADD_LIBRARY(libxmlrpc_xmlparse STATIC	${SOURCES})

SET(XMLRPC_LIBS ${XMLRPC_LIBS} libxmlrpc_xmlparse)

#libxmlrpc_xmltok Library
SET(CXXSOURCES	
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok/xmlrole.c
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok/xmltok.c
	)

SET(HXXSOURCES ${HXXSOURCES}
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok/asciitab.h
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok/iasciitab.h
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok/latin1tab.h
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok/nametab.h
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok/utf8tab.h
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok/xmldef.h
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok/xmlrole.h
	${CMAKE_CURRENT_SOURCE_DIR}/lib/expat/xmltok/xmltok.h
	#PARENT_SCOPE
	)

SET(INCLUDE_DIRS ${INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/lib/util/include
	${CMAKE_CURRENT_SOURCE_DIR}/include)
	
SET(ALL_HXXSOURCES ${ALL_HXXSOURCES} HXXSOURCES)
	
INCLUDE_DIRECTORIES(	
	${INCLUDE_DIRS}
	)

SET(SOURCES	${CXXSOURCES}	${HXXSOURCES}	)
SET(MYLIBRARIES	"")
	
ADD_LIBRARY(libxmlrpc_xmltok STATIC	${SOURCES})

SET(XMLRPC_LIBS ${XMLRPC_LIBS} libxmlrpc_xmltok PARENT_SCOPE)



