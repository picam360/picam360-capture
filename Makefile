OBJS=picam360_capture.o mrevent.o quaternion.o video.o mjpeg_decoder.o video_direct.o gl_program.o auto_calibration.o \
	omxcv_jpeg.o omxcv.o manual_mpu.o picam360_tools.o menu.o rtp.o \
	libs/freetypeGlesRpi/libFreetypeGlesRpi.a
PLUGINS=plugins/oculus_rift_dk2 plugins/mpu9250 plugins/kokuyoseki plugins/rov_agent
BIN=picam360-capture.bin

include Makefile.include


