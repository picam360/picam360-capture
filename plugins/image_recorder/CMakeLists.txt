cmake_minimum_required(VERSION 3.1.3)

message("image_recorder generating Makefile")
project(image_recorder)

find_package(PkgConfig REQUIRED)

add_library(image_recorder MODULE
	image_recorder.c
)

set_target_properties(image_recorder PROPERTIES
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

target_link_libraries(image_recorder
	${CMAKE_CURRENT_SOURCE_DIR}/../../libs/picam360-common/libpicam360-common.a
	pthread
	dl
)

#post build
add_custom_command(TARGET image_recorder POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:image_recorder> ${CMAKE_CURRENT_SOURCE_DIR}/../
)