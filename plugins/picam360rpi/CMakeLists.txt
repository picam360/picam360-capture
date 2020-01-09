cmake_minimum_required(VERSION 3.1.3)

message("picam360rpi generating Makefile")
project(picam360rpi)

find_package(PkgConfig REQUIRED)
#pkg_check_modules(OPENCV opencv REQUIRED)

find_file(BCM_HOST bcm_host.h /opt/vc/include)
if(BCM_HOST)
	message("RASPI")
	set( USE_GLES ON )
	set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/opt/vc/lib/pkgconfig")
	pkg_check_modules(BCMHOST bcm_host REQUIRED)
	add_definitions(-DBCM_HOST)
	add_definitions(-DOMX_SKIP64BIT) # this would be not need in the future
	
	include_directories( ${BCMHOST_INCLUDE_DIRS} )
	link_directories( ${BCMHOST_LIBRARY_DIRS} ) # need to upper of add_executable
	
	include_directories( /opt/vc/src/hello_pi/libs/ilclient )
	link_directories( /opt/vc/src/hello_pi/libs/ilclient ) # need to upper of add_executable
endif()

set(GLSL_HEADERS
  "glsl/picam360map_fsh.h"
  "glsl/picam360map_vsh.h"
  "glsl/window_fsh.h"
  "glsl/window_vsh.h"
)

add_library(picam360rpi MODULE
	create_plugin.cpp
	
	window_gl_renderer.c
#	picam360map_gl_renderer.c
#	equirectangular_gl_renderer.c
	
	mjpeg_omx_decoder.cpp
	
#	omx_encoder.cpp
#	omxcv_jpeg.cpp
#	omxcv.cpp

	${GLSL_HEADERS}
)

set_target_properties(picam360rpi PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
    PREFIX ""
)

#libomxjpeg
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/libomxjpeg)
add_dependencies(picam360rpi omxjpeg)
target_include_directories(picam360rpi PUBLIC lib/libomxjpeg)
target_link_libraries(picam360rpi omxjpeg)
link_directories (omxjpeg)

include_directories(
	./include
	../../include
	../../libs/picam360-common/include
#	${OPENCV_INCLUDE_DIRS}
)

#link_directories(
#	${OPENCV_LIBRARY_DIRS}
#)

target_link_libraries(picam360rpi
	pthread
	dl
#	${OPENCV_LIBRARIES}	
)

#packages
find_package(PkgConfig REQUIRED)

#opengl
if(USE_GLES)
	if(TEGRA)
		message("USE_VAO")
		add_definitions(-DUSE_VAO)
	endif()
	message("USE_GLES")
	add_definitions(-DUSE_GLES)
	
	if(BCM_HOST)
		pkg_check_modules(GLES brcmglesv2 REQUIRED)
		pkg_check_modules(EGL brcmegl REQUIRED)
	else()
		pkg_check_modules(GLES glesv2 REQUIRED)
		pkg_check_modules(EGL egl REQUIRED)
	endif()
	include_directories( ${GLES_INCLUDE_DIRS} ${EGL_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} )
else()
	find_package(OpenGL REQUIRED)
	pkg_check_modules(GLEW glew>=2.1 REQUIRED)
	pkg_check_modules(GLFW glfw3 REQUIRED)

	include_directories( ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} )
endif()

#glsl
add_custom_command(OUTPUT ${GLSL_HEADERS}
  COMMAND /usr/bin/xxd -i picam360map.fsh > picam360map_fsh.h
  COMMAND /usr/bin/xxd -i picam360map.vsh > picam360map_vsh.h
  COMMAND /usr/bin/xxd -i window.fsh > window_fsh.h
  COMMAND /usr/bin/xxd -i window.vsh > window_vsh.h
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/glsl"
  COMMENT "prepare glsl include files"
  VERBATIM
)


#post build
add_custom_command(TARGET picam360rpi POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:picam360rpi> ${CMAKE_CURRENT_SOURCE_DIR}/../
)