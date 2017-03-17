#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <dirent.h>
#include <stdbool.h>

#include "kokuyoseki.h"

static KOKUYOSEKI_CALLBACK lg_kokuyoseki_callback = NULL;

void set_kokuyoseki_callback(KOKUYOSEKI_CALLBACK callback) {
	lg_kokuyoseki_callback = callback;
}

static bool lg_stop_thread = false;
void *poling_thread_func(void* arg) {
	char *kokuyoseki_event = "/dev/input/event0";
	int fd = open(kokuyoseki_event, O_RDWR);
	if (fd < 0) {
		return NULL;
	}
	ioctl(fd, EVIOCGRAB, 1);
	while (!lg_stop_thread) {
		struct input_event event;

		if (read(fd, &event, sizeof(event)) != sizeof(event)) {
			perror("error : on read\n");
			return NULL;
		}
		if (event.type == 1) {
			if (lg_kokuyoseki_callback) {
				lg_kokuyoseki_callback(event.time, event.code, event.value);
			}
		}
	}
	return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Scan /dev looking for hidraw devices and then check to see if each is a kokuyoseki
/////////////////////////////////////////////////////////////////////////////////////////////
void open_kokuyoseki() {
	lg_stop_thread = false;
	static pthread_t poling_thread;
	pthread_create(&poling_thread, NULL, poling_thread_func, (void*) NULL);
	return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void close_kokuyoseki() {
	lg_stop_thread = true;
}
