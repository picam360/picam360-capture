#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <dirent.h>
#include <stdbool.h>
#include <pthread.h>

#include "kokuyoseki.h"

static KOKUYOSEKI_CALLBACK lg_kokuyoseki_callback = NULL;

void set_kokuyoseki_callback(KOKUYOSEKI_CALLBACK callback) {
	lg_kokuyoseki_callback = callback;
}

static int read_hex(const char * const filename) {
	FILE *in;
	unsigned int value;

	in = fopen(filename, "rb");
	if (!in)
		return -1;

	if (fscanf(in, "%x", &value) == 1) {
		fclose(in);
		return (int) value;
	}

	fclose(in);
	return -1;
}

static bool lg_stop_thread = false;
void *poling_thread_func(void* arg) {
	struct dirent *d;
	DIR *dir;
	char fileName[256];
	char *kokuyoseki_event = NULL;

	// Open /dev directory
	dir = opendir("/sys/class/input");

	// Iterate over /dev files
	while ((d = readdir(dir)) != 0) {
		int vendor = 0;
		int product = 0;
		{
			sprintf(fileName, "/sys/class/input/%s/device/id/vendor",
					d->d_name);
			vendor = read_hex(fileName);
		}
		{
			sprintf(fileName, "/sys/class/input/%s/device/id/product",
					d->d_name);
			product = read_hex(fileName);
		}

		if (vendor == KOKUYOSEKI_VENDOR && product == KOKUYOSEKI_PRODUCT) {
			sprintf(fileName, "/dev/input/%s", d->d_name);
			kokuyoseki_event = fileName;
			break;
		}
	}
	if (kokuyoseki_event == NULL) {
		return NULL;
	}
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
