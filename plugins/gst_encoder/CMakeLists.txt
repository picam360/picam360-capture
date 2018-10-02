cmake_minimum_required(VERSION 3.1.3)

message("gst_encoder generating Makefile")
project(gst_encoder)

add_library(gst_encoder MODULE
	gst_encoder.c
)
set_target_properties(gst_encoder PROPERTIES
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

target_link_libraries(gst_encoder
	pthread
	dl
)

#post build
add_custom_command(TARGET gst_encoder POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gst_encoder> ${CMAKE_CURRENT_SOURCE_DIR}/../
)