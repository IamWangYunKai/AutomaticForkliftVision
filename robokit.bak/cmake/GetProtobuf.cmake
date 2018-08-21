# protobuf 3.3.0

list(APPEND CMAKE_PREFIX_PATH ${LIBPROTOBUF_DIR})
set(Protobuf_SRC_ROOT_FOLDER ${LIBPROTOBUF_DIR})


IF(UNIX)
#set(TEMP_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})
#set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
ENDIF()

find_package(Protobuf REQUIRED)

IF(UNIX)
#set(CMAKE_FIND_LIBRARY_SUFFIXES ${TEMP_SUFFIXES})
#unset(TEMP_SUFFIXES)
ENDIF()

message("Protobuf_VERSION: " ${Protobuf_VERSION})
message("Protobuf_INCLUDE_DIR: " ${Protobuf_INCLUDE_DIR})
message("Protobuf_LIBRARY_DEBUG: " ${Protobuf_LIBRARY_DEBUG})
message("Protobuf_LIBRARY_RELEASE: " ${Protobuf_LIBRARY_RELEASE})
message("Protobuf_LITE_LIBRARY_DEBUG: " ${Protobuf_LITE_LIBRARY_DEBUG})
message("Protobuf_LITE_LIBRARY_RELEASE: " ${Protobuf_LITE_LIBRARY_RELEASE})
message("Protobuf_PROTOC_EXECUTABLE: " ${Protobuf_PROTOC_EXECUTABLE})
message("Protobuf_LIBRARIES: " ${Protobuf_LIBRARIES})

message("")
