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

#define NEXT_BUTTON 109
#define NEXT_BUTTON_LONG 63
#define BACK_BUTTON 104
#define BACK_BUTTON_LONG 1
#define BLACKOUT_BUTTON 48

#define BUTTON_RELEASE 0
#define BUTTON_PUSH 1
#define BUTTON_KEEP 0 2

typedef void (*KOKUYOSEKI_CALLBACK)(struct timeval time, int button, int value);
void set_kokuyoseki_callback(KOKUYOSEKI_CALLBACK callback);
void open_kokuyoseki();
void close_kokuyoseki();

void create_kokuyoseki(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin);
