cmake_minimum_required(VERSION 3.1.3)

message("video_reciever generating Makefile")
project(view_point_multiplexor)

find_package(PkgConfig REQUIRED)

add_library(view_point_multiplexor MODULE
	view_point_multiplexor.c
)

set_target_properties(view_point_multiplexor PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
    PREFIX ""
)
	
include_directories(
	../../include
)
link_directories(
)

target_link_libraries(view_point_multiplexor
	${CMAKE_CURRENT_SOURCE_DIR}/../../libs/picam360-common/libpicam360-common.a
	pthread
	dl
)

#post build
add_custom_command(TARGET view_point_multiplexor POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:view_point_multiplexor> ${CMAKE_CURRENT_SOURCE_DIR}/../
)