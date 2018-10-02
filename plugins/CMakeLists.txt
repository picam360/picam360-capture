cmake_minimum_required(VERSION 3.1.3)

message("plugins generating Makefile")

find_package(PkgConfig REQUIRED)

find_file(BCM_HOST bcm_host.h /opt/vc/include)
find_file(TEGRA tegra_drm.h /usr/include/drm)

if(BCM_HOST)
	message("RASPI")
	add_subdirectory(v4l2_capture)
	add_subdirectory(mjpeg_omx_decoder)
	add_subdirectory(omx_encoder)
elseif(TEGRA)
	message("JETSON")
elseif(APPLE)
	message("OSX")
	add_subdirectory(ffmpeg_capture)
elseif(UNIX)
	message("UNIX or LINUX")
elseif(WIN32)
	message("WINDOWS")
endif()

#common
add_subdirectory(mpu9250)
add_subdirectory(video_transmitter)
add_subdirectory(video_reciever)
add_subdirectory(window_renderer)
add_subdirectory(equirectangular_renderer)
add_subdirectory(h265_encoder)