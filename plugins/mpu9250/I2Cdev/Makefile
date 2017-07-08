CC=gcc
AR=ar
CCFLAGS=-Wall -g -O2
COPTS=-c
LDFLAGS= -lm -lrt

LIB=../libI2Cdev.a
OBJ=I2Cdev.o

%.o: %.c                                                                         
	$(CC) $(CFLAGS) $(COPTS) $< -o $@ 

$(LIB): $(OBJ)
	$(AR) rcs $(LIB) $(OBJ)

all: $(LIB)

clean:
	rm -rf *.o
	rm -rf $(LIB)
