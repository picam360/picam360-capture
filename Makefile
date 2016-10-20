OBJS=triangle.o video.o gl_program.o device.o omxcv_jpeg.o omxcv.o picam360_tools.o MotionSensor/libMotionSensor.a libs/libI2Cdev.a
BIN=picam360-oculus-viewer.bin
LDFLAGS+=-lilclient -ljansson

include Makefile.include


