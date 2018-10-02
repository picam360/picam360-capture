cmake_minimum_required(VERSION 3.1.3)

message("equirectangular_renderer generating Makefile")
project(equirectangular_renderer)

find_package(PkgConfig REQUIRED)

find_file(BCM_HOST bcm_host.h /opt/vc/include)
if(BCM_HOST)
	message("RASPI")
	set( USE_GLES ON )
	set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/opt/vc/lib/pkgconfig")
	pkg_check_modules(BCMHOST bcm_host REQUIRED)
	add_definitions(-DBCM_HOST)
	include_directories( ${BCMHOST_INCLUDE_DIRS} )
	link_directories( ${BCMHOST_LIBRARY_DIRS} ) # need to upper of add_executable
endif()

find_file(TEGRA tegra_drm.h /usr/include/drm)
if(TEGRA)
	message("TEGRA")
	set( USE_GLES ON )
	add_definitions(-DTEGRA)
endif()

add_library(equirectangular_renderer MODULE
	equirectangular_renderer.c
)
set_target_properties(equirectangular_renderer PROPERTIES
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
	../../
)
link_directories(
)

target_link_libraries(equirectangular_renderer
	pthread
	dl
)

#packages
find_package(PkgConfig REQUIRED)

#opengl
if(USE_GLES)
	message("USE_GLES")
	add_definitions(-DUSE_GLES)
	
	pkg_check_modules(GLES glesv2 REQUIRED)
	pkg_check_modules(EGL egl REQUIRED)
	include_directories( ${GLES_INCLUDE_DIRS} ${EGL_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} )
else()
	find_package(OpenGL REQUIRED)
	pkg_check_modules(GLEW glew>=2.1 REQUIRED)
	pkg_check_modules(GLFW glfw3 REQUIRED)
		
	include_directories( ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} )
endif()

#post build
add_custom_command(TARGET equirectangular_renderer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:equirectangular_renderer> ${CMAKE_CURRENT_SOURCE_DIR}/../
)