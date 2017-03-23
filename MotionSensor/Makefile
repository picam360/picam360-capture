CC=gcc
CXX=g++
AR=ar
CFLAGS=-Wall -g -O2
CXX_OPTS=-c -DMPU9250 -DMPU_DEBUGOFF -I../libs/

OBJECTS=sensor.o inv_mpu_lib/inv_mpu.o inv_mpu_lib/inv_mpu_dmp_motion_driver.o

LIB=libMotionSensor.a

%.o: %.c
	$(CC) -std=gnu11 $(CFLAGS) $(CXX_OPTS) $< -o $@ 
	
%.o: %.cc
	$(CXX) -std=c++11 $(CFLAGS) $(CXX_OPTS) $< -o $@ 
	

$(LIB): $(OBJECTS) 
	$(AR) rcs $(LIB) $(OBJECTS) 

all: $(LIB)

clean:
	rm -rf *.o
	rm -rf inv_mpu_lib/*.o
	rm -rf $(LIB)
