cmake_minimum_required(VERSION 3.1.3)

message("rov_agent generating Makefile")
project(rov_agent)

find_package(PkgConfig REQUIRED)

add_library(rov_agent MODULE
	rov_agent.c
)
set_target_properties(rov_agent PROPERTIES
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
	../..
)
link_directories(
)

target_link_libraries(rov_agent
	pthread
	dl
)

#post build
add_custom_command(TARGET rov_agent POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:rov_agent> ${CMAKE_CURRENT_SOURCE_DIR}/../
)