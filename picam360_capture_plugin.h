#pragma once

#include <stdint.h>
#include <pthread.h>
#include <jansson.h>//json parser
#include "menu.h"

enum PICAM360_CAPTURE_EVENT{
	PICAM360_CAPTURE_EVENT_AFTER_FRAME,
};

typedef void (*PICAM360_CAPTURE_CALLBACK)(enum PICAM360_CAPTURE_EVENT event);

typedef struct _PLUGIN_HOST_T{
	float *(*get_view_quatanion)();
	void (*set_view_quatanion)(float *value);
	float *(*get_view_compass)();
	void (*set_view_compass)(float *value);
	float (*get_view_temperature)();
	void (*set_view_temperature)(float value);
	float (*get_view_north)();
	void (*set_view_north)(float value);

	float *(*get_camera_quatanion)(int cam_num);
	void (*set_camera_quatanion)(int cam_num, float *value);
	float *(*get_camera_compass)();
	void (*set_camera_compass)(float *value);
	float (*get_camera_temperature)();
	void (*set_camera_temperature)(float value);
	float (*get_camera_north)();
	void (*set_camera_north)(float value);

	void (*decode_video)(int cam_num, unsigned char *data, int data_len);
	void (*lock_texture)();
	void (*unlock_texture)();
	void (*set_cam_texture_cur)(int cam_num, int cur);
	void (*get_texture_size)(uint32_t *width_out, uint32_t *height_out);

	MENU_T *(*get_menu)();
	bool (*get_menu_visible)();
	void (*set_menu_visible)(bool value);

	float (*get_fov)();
	void (*set_fov)(float value);

	void (*add_event_handler)(PICAM360_CAPTURE_CALLBACK callback);
	void (*send_command)(char *cmd);
} PLUGIN_HOST_T;

typedef struct _PLUGIN_T{
	char name[64];
	void (*release)(void *user_data);
	void (*command_handler)(void *user_data, char *cmd);
	void (*init_options)(void *user_data, json_t *options);
	void (*save_options)(void *user_data, json_t *options);
	wchar_t *(*get_info)(void *user_data);
	void *user_data;
} PLUGIN_T;

typedef void (*CREATE_PLUGIN)(PLUGIN_HOST_T *plugin_host, PLUGIN_T **plugin);

