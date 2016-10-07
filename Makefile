OBJS=triangle.o video.o gl_program.o device.o MotionSensor/libMotionSensor.a libs/libI2Cdev.a libs/geodesic.a
BIN=picam360-oculus-viewer.bin
LDFLAGS+=-lilclient

include Makefile.include


