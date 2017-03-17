#pragma once

#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

#ifndef HIDIOCSFEATURE
#error Update Kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>


#define KOKUYOSEKI_VENDOR 0x062a
#define KOKUYOSEKI_PRODUCT 0x412c

void open_kokuyoseki();
void close_kokuyoseki();
