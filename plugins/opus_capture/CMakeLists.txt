cmake_minimum_required(VERSION 3.1.3)

message("opus_capture generating Makefile")
project(opus_capture)

add_library(opus_capture MODULE
	opus_capture.c
)
set_target_properties(opus_capture PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
    PREFIX ""
)
	
include_directories(
	../../libs/picam360-common/include
	../../include
)
link_directories(
)

target_link_libraries(opus_capture
	pthread
	dl
)

#packages
find_package(PkgConfig REQUIRED)


#post build
add_custom_command(TARGET opus_capture POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:opus_capture> ${CMAKE_CURRENT_SOURCE_DIR}/../
)