cmake_minimum_required(VERSION 3.1.3)

project(h265_encoder)

add_library(h265_encoder MODULE
	h265_encoder.c
)
set_target_properties(h265_encoder PROPERTIES
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

target_link_libraries(h265_encoder
	pthread
	dl
)

#post build
add_custom_command(TARGET h265_encoder POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:h265_encoder> ${CMAKE_CURRENT_SOURCE_DIR}/../
)