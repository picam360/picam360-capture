cmake_minimum_required(VERSION 3.1.3)

project(omx_encoder)

add_library(omx_encoder MODULE
	omx_encoder.cc
	omxcv_jpeg.cc
	omxcv.cc
)
set_target_properties(omx_encoder PROPERTIES
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

target_link_libraries(omx_encoder
	pthread
	dl
)

#post build
add_custom_command(TARGET omx_encoder POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:omx_encoder> ${CMAKE_CURRENT_SOURCE_DIR}/../
)