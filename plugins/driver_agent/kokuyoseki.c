#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <dirent.h>

#include "kokuyoseki.h"

static BOOLEAN is_kokuyoseki(const char *path);

/////////////////////////////////////////////////////////////////////////////////////////////
// Scan /dev looking for hidraw devices and then check to see if each is a kokuyoseki
/////////////////////////////////////////////////////////////////////////////////////////////
void open_kokuyoseki() {
	struct dirent *d;
	DIR *dir;
	char fileName[32];

	// Open /dev directory
	dir = opendir("/dev");

	// Iterate over /dev files
	while ((d = readdir(dir)) != 0) {
		// Is this a hidraw device?
		if (strstr(d->d_name, "hidraw")) {
			sprintf(fileName, "/dev/%s", d->d_name);
			if (is_kokuyoseki(fileName)) {
			}
		}
	}
	closedir(dir);
	return dev;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void close_kokuyoseki() {
	// TODO - clean up device
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Open the device and check the vendor and product codes to see if it's a kokuyoseki
/////////////////////////////////////////////////////////////////////////////////////////////
BOOLEAN is_kokuyoseki(const char *path) {
	int fd;
	int res;
	struct hidraw_devinfo info;

	// Open the device
	fd = open(path, O_RDWR);
	if (fd < 0) {
		perror("Unable to open device");
		return FALSE;
	}
	// Get USB info
	res = ioctl(fd, HIDIOCGRAWINFO, &info);
	close(fd);
	if (res < 0) {
		perror("HIDIOCGRAWINFO");
		return FALSE;
	} else {
		// Check to see if the vendor and product match
		if (info.vendor == KOKUYOSEKI_VENDOR
				&& info.product == KOKUYOSEKI_PRODUCT) {
			return TRUE;
		}
	}
	return FALSE;
}
