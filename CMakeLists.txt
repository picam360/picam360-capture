cmake_minimum_required(VERSION 3.1.3)

project(picam360-capture)

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

pkg_check_modules(JANSSON jansson REQUIRED)
pkg_check_modules(LIBEDIT libedit REQUIRED)
pkg_check_modules(OPENCV opencv REQUIRED)

include_directories(
	include
	${JANSSON_INCLUDE_DIRS}
	${LIBEDIT_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
)
link_directories(
	${JANSSON_LIBRARY_DIRS}
	${LIBEDIT_LIBRARY_DIRS}
	${OPENCV_LIBRARY_DIRS}
)

#opengl
if(USE_GLES)
	message("USE_GLES")
	add_definitions(-DUSE_GLES)
	
	pkg_check_modules(GLES glesv2 REQUIRED)
	pkg_check_modules(EGL egl REQUIRED)
	pkg_check_modules(FREETYPE freetype2 REQUIRED)
	include_directories( ${GLES_INCLUDE_DIRS} ${EGL_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} )
	include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetypeGlesRpi/src )
else()
	find_package(OpenGL REQUIRED)
	pkg_check_modules(GLEW glew>=2.1 REQUIRED)
	pkg_check_modules(GLFW glfw3 REQUIRED)
	pkg_check_modules(FREETYPE freetype2 REQUIRED)
		
	include_directories( ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} )
	include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetype-gl )
	
	link_directories( ${OPENGL_LIBRARY_DIRS} ${GLEW_LIBRARY_DIRS} ${GLFW_LIBRARY_DIRS} ${FREETYPE_LIBRARY_DIRS} )
endif()

#add_executable 
#target_link_libraries should be define below here
add_executable(picam360-capture.bin
	picam360_capture.c
	rtp.cc
	mrevent.c
	quaternion.c
	manual_mpu.c
	gl_program.cc
	auto_calibration.cc
	manual_mpu.c
	picam360_tools.cc
	menu.c
)

if(BCM_HOST)
	target_link_libraries(picam360-capture.bin ${BCMHOST_LIBRARIES} )
endif()

if(APPLE)
	message("OSX")
elseif(UNIX)
	message("UNIX or LINUX")
	#add_definitions(-DENABLE_V4L2)
elseif(WIN32)
	message("WINDOWS")
endif()

set_target_properties(picam360-capture.bin PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
)

target_link_libraries(picam360-capture.bin
	${JANSSON_LIBRARIES}
	${LIBEDIT_LIBRARIES}
	${OPENCV_LIBRARIES}
	pthread
	dl
)

#opengl
if(USE_GLES)
	target_link_libraries(picam360-capture.bin ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetypeGlesRpi/libFreetypeGlesRpi.a)
	target_link_libraries(picam360-capture.bin ${GLES_LIBRARIES} ${EGL_LIBRARIES} ${FREETYPE_LIBRARIES} )
else()
	target_link_libraries(picam360-capture.bin ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetype-gl/libfreetype-gl.a )
	target_link_libraries(picam360-capture.bin ${OPENGL_LIBRARIES} ${GLEW_STATIC_LIBRARIES} ${GLFW_STATIC_LIBRARIES} ${FREETYPE_LIBRARIES})
endif()

option(USE_MPU9250 "Use mpu9250" OFF)
option(USE_ROV_DRIVER "Use rov_driver" OFF)

#plugins
if(USE_MPU9250)
	message("enable mpu9250")
	add_subdirectory(plugins/mpu9250)
endif()

if(USE_ROV_DRIVER)
	message("enable rov_driver")
	add_subdirectory(plugins/rov_driver)
endif()