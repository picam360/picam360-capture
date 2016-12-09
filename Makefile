OBJS=picam360_capture.o video.o video_mjpeg.o video_direct.o gl_program.o device.o omxcv_jpeg.o omxcv.o picam360_tools.o MotionSensor/libMotionSensor.a libs/libI2Cdev.a
BIN=picam360-capture.bin
LDFLAGS+=-lilclient -ljansson

include Makefile.include


