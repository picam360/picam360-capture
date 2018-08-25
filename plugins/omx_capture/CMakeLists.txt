cmake_minimum_required(VERSION 3.1.3)

project(omx_capture)

add_library(omx_capture MODULE
	omx_capture.c
)
set_target_properties(omx_capture PROPERTIES
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

target_link_libraries(omx_capture
	pthread
	dl
)

#post build
add_custom_command(TARGET omx_capture POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:omx_capture> ${CMAKE_CURRENT_SOURCE_DIR}/../
)