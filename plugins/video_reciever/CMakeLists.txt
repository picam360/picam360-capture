cmake_minimum_required(VERSION 3.1.3)

project(video_reciever)

find_package(PkgConfig REQUIRED)

add_library(video_reciever MODULE
	video_reciever.c
)

set_target_properties(video_reciever PROPERTIES
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

target_link_libraries(video_reciever
	pthread
	dl
)

#post build
add_custom_command(TARGET video_reciever POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:video_reciever> ${CMAKE_CURRENT_SOURCE_DIR}/../
)