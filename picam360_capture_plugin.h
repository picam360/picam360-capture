#pragma once

#include <stdint.h>
#include <pthread.h>
#include <jansson.h>//json parser
#include "menu.h"

enum PICAM360_CAPTURE_EVENT {
	PICAM360_CAPTURE_EVENT_AFTER_FRAME, PICAM360_CAPTURE_EVENT_AFTER_SNAP, PICAM360_CAPTURE_EVENT_TEXTURE0_UPDATED,PICAM360_CAPTURE_EVENT_TEXTURE1_UPDATED,
};
enum RENDERING_MODE {
	RENDERING_MODE_WINDOW, RENDERING_MODE_EQUIRECTANGULAR, RENDERING_MODE_FISHEYE,
};

#define PICAM360_HOST_NODE_ID 0

typedef struct _MPU_T {
	char name[64];
	void (*release)(void *user_data);
	VECTOR4D_T (*get_quatanion)(void *user_data);
	VECTOR4D_T (*get_compass)(void *user_data);
	float (*get_temperature)(void *user_data);
	float (*get_north)(void *user_data);
	void *user_data;
} MPU_T;

typedef struct _PLUGIN_HOST_T {
	VECTOR4D_T (*get_view_quatanion)();
	VECTOR4D_T (*get_view_compass)();
	float (*get_view_temperature)();
	float (*get_view_north)();

	VECTOR4D_T (*get_camera_quatanion)(int cam_num);
	void (*set_camera_quatanion)(int cam_num, VECTOR4D_T value);
	VECTOR4D_T (*get_camera_compass)();
	void (*set_camera_compass)(VECTOR4D_T value);
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

	void (*send_command)(const char *cmd);
	void (*send_event)(uint32_t node_id, uint32_t event_id);
	void (*add_mpu)(MPU_T *mpu);

	void (*snap)(uint32_t width, uint32_t height, enum RENDERING_MODE mode, const char *path);
} PLUGIN_HOST_T;

typedef struct _PLUGIN_T {
	char name[64];
	void (*release)(void *user_data);
	void (*command_handler)(void *user_data, const char *cmd);
	void (*event_handler)(void *user_data, uint32_t node_id, uint32_t event_id);
	void (*init_options)(void *user_data, json_t *options);
	void (*save_options)(void *user_data, json_t *options);
	wchar_t *(*get_info)(void *user_data);
	void *user_data;
	uint32_t node_id;
} PLUGIN_T;

typedef void (*CREATE_PLUGIN)(PLUGIN_HOST_T *plugin_host, PLUGIN_T **plugin);

