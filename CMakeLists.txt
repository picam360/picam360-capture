cmake_minimum_required(VERSION 3.1.3)

project(picam360-capture)

#check raspberry pi
include(CheckIncludeFile)
CHECK_INCLUDE_FILE(/opt/vc/include/bcm_host.h BCMHOST)

if(APPLE)
	message("OSX")
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
elseif(UNIX)
	message("UNIX or LINUX")
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

find_package(PkgConfig REQUIRED)
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
target_link_libraries(picam360-capture.bin
	${JANSSON_LIBRARIES}
	${LIBEDIT_LIBRARIES}
	${OPENCV_LIBRARIES}
	pthread
	dl
)

#opengl
if(USE_GLES)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetypeGlesRpi/src )
else()
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
pkg_check_modules(GLFW glfw3 REQUIRED)
find_package(Freetype REQUIRED)
	
include_directories( ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} )
target_link_libraries(picam360-capture.bin ${OPENGL_LIBRARIES} ${GLEW_LIBRARY} ${GLFW_LIBRARIES} ${FREETYPE_LIBRARIES})

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetype-gl )
target_link_libraries(picam360-capture.bin ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetype-gl/libfreetype-gl.a )
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