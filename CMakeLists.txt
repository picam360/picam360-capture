cmake_minimum_required(VERSION 3.1.3)

message("picam360-capture.bin generating Makefile")
project(picam360-capture.bin)

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

set(IMAGE_HEADERS
  "img/logo_png.h"
  "img/calibration_png.h"
)

set(GLSL_HEADERS
  "glsl/board_fsh.h"
  "glsl/board_vsh.h"
  "glsl/calibration_fsh.h"
  "glsl/calibration_vsh.h"
  "glsl/freetype_fsh.h"
  "glsl/freetype_vsh.h"
)

#add_executable 
#target_link_libraries should be define below here
add_executable(picam360-capture.bin
	src/picam360_capture.c
	src/manual_mpu.c
	src/menu.c
	src/board_renderer.c
	src/calibration_renderer.c
	${IMAGE_HEADERS}
	${GLSL_HEADERS}
)

#picam360-common
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs/picam360-common)
add_dependencies(picam360-capture.bin picam360-common)
target_include_directories(picam360-common PUBLIC libs/picam360-common)
target_link_libraries(picam360-capture.bin picam360-common)
link_directories (libs/picam360-common)

pkg_check_modules(JANSSON jansson REQUIRED)
pkg_check_modules(LIBEDIT libedit REQUIRED)

include_directories(
	./
	./include
	./libs/picam360-common/include
	${JANSSON_INCLUDE_DIRS}
	${LIBEDIT_INCLUDE_DIRS}
)
link_directories(
	${JANSSON_LIBRARY_DIRS}
	${LIBEDIT_LIBRARY_DIRS}
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
	pkg_check_modules(FREETYPE freetype2 REQUIRED)
	include_directories( ${GLES_INCLUDE_DIRS} ${EGL_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} )
	include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetypeGlesRpi/src )

	add_custom_target(freetypeGlesRpi_make
		COMMAND make -C libs/freetypeGlesRpi
	)
	add_dependencies(picam360-capture.bin freetypeGlesRpi_make)
else()
	find_package(OpenGL REQUIRED)
	pkg_check_modules(GLEW glew>=2.1 REQUIRED)
	pkg_check_modules(GLFW glfw3 REQUIRED)
	pkg_check_modules(FREETYPE freetype2 REQUIRED)
		
	include_directories( ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${FREETYPE_INCLUDE_DIRS} )
	include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetype-gl )
	
	link_directories( ${OPENGL_LIBRARY_DIRS} ${GLEW_LIBRARY_DIRS} ${GLFW_LIBRARY_DIRS} ${FREETYPE_LIBRARY_DIRS} )

	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs/freetype-gl)
	add_dependencies(picam360-capture.bin freetype-gl)
endif()

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

#img
add_custom_command(OUTPUT ${IMAGE_HEADERS}
  COMMAND /usr/bin/xxd -i logo.png > logo_png.h
  COMMAND /usr/bin/xxd -i calibration.png > calibration_png.h
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/img"
  COMMENT "prepare img include files"
  VERBATIM
)

#glsl
add_custom_command(OUTPUT ${GLSL_HEADERS}
  COMMAND /usr/bin/xxd -i board.fsh > board_fsh.h
  COMMAND /usr/bin/xxd -i board.vsh > board_vsh.h
  COMMAND /usr/bin/xxd -i calibration.fsh > calibration_fsh.h
  COMMAND /usr/bin/xxd -i calibration.vsh > calibration_vsh.h
  COMMAND /usr/bin/xxd -i freetype.fsh > freetype_fsh.h
  COMMAND /usr/bin/xxd -i freetype.vsh > freetype_vsh.h
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/glsl"
  COMMENT "prepare glsl include files"
  VERBATIM
)

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
	pthread
	dl
	uuid
)

#opengl
if(USE_GLES)
	target_link_libraries(picam360-capture.bin ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetypeGlesRpi/libFreetypeGlesRpi.a)
	target_link_libraries(picam360-capture.bin ${GLES_LIBRARIES} ${EGL_LIBRARIES} ${FREETYPE_LIBRARIES} )
else()
	target_link_libraries(picam360-capture.bin ${CMAKE_CURRENT_SOURCE_DIR}/libs/freetype-gl/libfreetype-gl.a )
	target_link_libraries(picam360-capture.bin ${OPENGL_LIBRARIES} ${GLEW_STATIC_LIBRARIES} ${GLFW_STATIC_LIBRARIES} ${FREETYPE_LIBRARIES})
endif()

#install
configure_file( setup/picam360-capture.in setup/picam360-capture )
configure_file( setup/picam360-capture.service.in setup/picam360-capture.service )
configure_file( setup/install.sh.in setup/install.sh )
install(CODE "execute_process(COMMAND bash -x setup/install.sh)")

#plugins
add_subdirectory(plugins)
if(EXISTS advance_plugins)
	add_subdirectory(advance_plugins)
endif()