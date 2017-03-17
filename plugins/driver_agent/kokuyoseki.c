#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <dirent.h>
#include <stdbool.h>

#include "kokuyoseki.h"

static bool lg_stop_thread = false;
void *poling_thread_func(void* arg) {
	char *kokuyoseki_event = "/dev/input/event0";
	int fd = open(kokuyoseki_event, O_RDWR);
	ioctl(fd, EVIOCGRAB, 1);
	if (fd < 0) {
		return;
	}
	while (!lg_stop_thread) {
		struct input_event event;

		if (read(mousefd, &event, sizeof(event)) != sizeof(event)) {
			perror("error : on read\n");
			return;
		}
	}
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
