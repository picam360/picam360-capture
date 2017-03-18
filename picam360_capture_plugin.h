#pragma once

#include "picam360_capture_plugin.h"

typedef struct _PLUGIN_HOST_T{
	float *(*get_view_quatanion)();
	void (*set_view_quatanion)(float *value);
	float *(*get_camera_quatanion)();
	void (*set_camera_quatanion)(float *value);
} PLUGIN_HOST_T;

typedef struct _PLUGIN_T{
	char name[64];
	void (*release)(void *user_data);
	void (*command_handler)(void *user_data, char *cmd);
	void *user_data;
} PLUGIN_T;

typedef void (*CREATE_PLUGIN)(PLUGIN_HOST_T *plugin_host, PLUGIN_T **plugin);

