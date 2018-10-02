cmake_minimum_required(VERSION 3.1.3)

message("v4l2_capture generating Makefile")
project(v4l2_capture)

find_package(PkgConfig REQUIRED)

add_library(v4l2_capture MODULE
	v4l2_capture.cc
	v4l2_handler.c
)
set_target_properties(v4l2_capture PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
    PREFIX ""
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
	
include_directories(
	../../include
	../..
)
link_directories(
)

target_link_libraries(v4l2_capture
	pthread
	dl
)

#post build
add_custom_command(TARGET v4l2_capture POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:v4l2_capture> ${CMAKE_CURRENT_SOURCE_DIR}/../
)