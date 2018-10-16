cmake_minimum_required(VERSION 3.1.3)

message("picam360-common generating Makefile")
project(picam360-common)

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

#add_executable 
#target_link_libraries should be define below here
add_library(picam360-common STATIC
	src/rtp.cc
	src/mrevent.c
	src/quaternion.c
	src/gl_program.cc
)

include_directories(
	./
	./include
)
link_directories(
)

#opengl
if(USE_GLES)
	message("USE_GLES")
	add_definitions(-DUSE_GLES)
	
	if(BCM_HOST)
		pkg_check_modules(GLES brcmglesv2 REQUIRED)
		pkg_check_modules(EGL brcmegl REQUIRED)
	else()
		pkg_check_modules(GLES glesv2 REQUIRED)
		pkg_check_modules(EGL egl REQUIRED)
	endif()
else()
	find_package(OpenGL REQUIRED)
	pkg_check_modules(GLEW glew>=2.1 REQUIRED)
	pkg_check_modules(GLFW glfw3 REQUIRED)
endif()

if(BCM_HOST)
	target_link_libraries(picam360-common ${BCMHOST_LIBRARIES} )
endif()

if(APPLE)
	message("OSX")
elseif(UNIX)
	message("UNIX or LINUX")
	#add_definitions(-DENABLE_V4L2)
elseif(WIN32)
	message("WINDOWS")
endif()

set_target_properties(picam360-common PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
    POSITION_INDEPENDENT_CODE TRUE
)

target_link_libraries(picam360-common
	pthread
	dl
)

#opengl
if(USE_GLES)
	target_link_libraries(picam360-common ${GLES_LIBRARIES} ${EGL_LIBRARIES})
else()
	target_link_libraries(picam360-common ${OPENGL_LIBRARIES} ${GLEW_STATIC_LIBRARIES} ${GLFW_STATIC_LIBRARIES})
endif()