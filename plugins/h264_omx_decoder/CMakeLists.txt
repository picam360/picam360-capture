cmake_minimum_required(VERSION 3.1.3)

project(h264_omx_decoder)

add_library(h264_omx_decoder MODULE
	h264_omx_decoder.c
)
set_target_properties(h264_omx_decoder PROPERTIES
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

target_link_libraries(h264_omx_decoder
	pthread
	dl
)

#post build
add_custom_command(TARGET h264_omx_decoder POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:h264_omx_decoder> ${CMAKE_CURRENT_SOURCE_DIR}/../
)