OBJS=picam360_capture.o mrevent.o quaternion.o video.o mjpeg_decoder.o video_direct.o gl_program.o auto_calibration.o \
	omxcv_jpeg.o omxcv.o manual_mpu.o picam360_tools.o menu.o \
	libs/MotionSensor/libMotionSensor.a libs/libI2Cdev.a libs/freetypeGlesRpi/libFreetypeGlesRpi.a \
	plugins/driver_agent/driver_agent.o plugins/driver_agent/kokuyoseki.o plugins/driver_agent/rtp.o \
	plugins/mpu9250/mpu9250.o plugins/oculus_rift_dk2/oculus_rift_dk2.o
BIN=picam360-capture.bin
LDFLAGS+=-lilclient -ljansson

include Makefile.include


