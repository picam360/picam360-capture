cmake_minimum_required(VERSION 3.1.3)

message("video_reciever generating Makefile")
project(rtp_streamer)

find_package(PkgConfig REQUIRED)

add_library(rtp_streamer MODULE
	rtp_streamer.c
)

set_target_properties(rtp_streamer PROPERTIES
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

target_link_libraries(rtp_streamer
	${CMAKE_CURRENT_SOURCE_DIR}/../../libs/picam360-common/libpicam360-common.a
	pthread
	dl
)

#post build
add_custom_command(TARGET rtp_streamer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:rtp_streamer> ${CMAKE_CURRENT_SOURCE_DIR}/../
)