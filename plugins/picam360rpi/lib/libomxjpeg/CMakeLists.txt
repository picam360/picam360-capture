cmake_minimum_required(VERSION 3.1.3)

message("omxjpeg generating Makefile")
project(omxjpeg)

find_package(PkgConfig REQUIRED)

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

#add_executable 
#target_link_libraries should be define below here
add_library(omxjpeg STATIC
	omxjpeg.c
	omx_jpeg_decompress.c
)

include_directories(
	./
)
link_directories(
)

#egl
if(BCM_HOST)
	pkg_check_modules(EGL brcmegl REQUIRED)
else()
	pkg_check_modules(EGL egl REQUIRED)
endif()

if(BCM_HOST)
	target_link_libraries(omxjpeg ${BCMHOST_LIBRARIES} )
	target_link_libraries(omxjpeg ilclient openmaxil )
endif()

if(APPLE)
	message("OSX")
elseif(UNIX)
	message("UNIX or LINUX")
	#add_definitions(-DENABLE_V4L2)
elseif(WIN32)
	message("WINDOWS")
endif()

set_target_properties(omxjpeg PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
    POSITION_INDEPENDENT_CODE TRUE
)

target_link_libraries(omxjpeg
	pthread
	dl
)

target_link_libraries(omxjpeg ${EGL_LIBRARIES})