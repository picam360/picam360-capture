cmake_minimum_required(VERSION 3.1.3)

message("process_streamer generating Makefile")
project(process_streamer)

find_package(PkgConfig REQUIRED)

add_library(process_streamer MODULE
	process_streamer.cpp
)
set_target_properties(process_streamer PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
    PREFIX ""
)
	
include_directories(
	../../
	../../include
	../../libs/picam360-common/include
)
link_directories(
)

target_link_libraries(process_streamer
	${CMAKE_CURRENT_SOURCE_DIR}/../../libs/picam360-common/libpicam360-common.a
	pthread
	dl
)

if(APPLE)
	set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -lc++")
endif()

#packages
find_package(PkgConfig REQUIRED)


#post build
add_custom_command(TARGET process_streamer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:process_streamer> ${CMAKE_CURRENT_SOURCE_DIR}/../
)