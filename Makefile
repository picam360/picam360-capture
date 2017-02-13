OBJS=picam360_capture.o mrevent.o video.o video_mjpeg.o video_direct.o gl_program.o auto_calibration.o \
	omxcv_jpeg.o omxcv.o status_watcher.o device.o view_coordinate_mpu9250.o picam360_tools.o \
	MotionSensor/libMotionSensor.a libs/libI2Cdev.a
BIN=picam360-capture.bin
LDFLAGS+=-lilclient -ljansson

include Makefile.include


