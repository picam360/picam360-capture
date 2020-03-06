cmake_minimum_required(VERSION 3.1.3)

message("ffmpeg_capture generating Makefile")
project(ffmpeg_capture)

find_package(PkgConfig REQUIRED)

add_library(ffmpeg_capture MODULE
	ffmpeg_capture.cpp
)
set_target_properties(ffmpeg_capture PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
    PREFIX ""
)
	
include_directories(
	../../
	../../include
	../../libs/picam360-common/include
)
link_directories(
)

target_link_libraries(ffmpeg_capture
	${CMAKE_CURRENT_SOURCE_DIR}/../../libs/picam360-common/libpicam360-common.a
	pthread
	dl
)

if(APPLE)
	set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -lc++")
endif()

#packages
find_package(PkgConfig REQUIRED)


#post build
add_custom_command(TARGET ffmpeg_capture POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ffmpeg_capture> ${CMAKE_CURRENT_SOURCE_DIR}/../
)