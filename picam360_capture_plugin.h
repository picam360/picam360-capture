#pragma once

#include <stdint.h>
#include <pthread.h>
#include <jansson.h>//json parser
#include "quaternion.h"
#include "menu.h"

//0x00** is reserved by system
#define PICAM360_HOST_NODE_ID 0x0000
#define PICAM360_CONTROLLER_NODE_ID 0x0001

#define UPSTREAM_DOMAIN "upstream."
#define UPSTREAM_DOMAIN_SIZE (sizeof(UPSTREAM_DOMAIN) - 1)

enum PICAM360_CAPTURE_EVENT {
	PICAM360_CAPTURE_EVENT_AFTER_FRAME, PICAM360_CAPTURE_EVENT_AFTER_SNAP, PICAM360_CAPTURE_EVENT_TEXTURE0_UPDATED, PICAM360_CAPTURE_EVENT_TEXTURE1_UPDATED,
};
enum RENDERING_MODE {
	RENDERING_MODE_WINDOW, RENDERING_MODE_EQUIRECTANGULAR, RENDERING_MODE_FISHEYE,
};

enum PICAM360_CONTROLLER_EVENT {
	PICAM360_CONTROLLER_EVENT_NONE, PICAM360_CONTROLLER_EVENT_NEXT, PICAM360_CONTROLLER_EVENT_BACK,
};

typedef struct _MPU_T {
	char name[64];
	void (*release)(void *user_data);
	VECTOR4D_T (*get_quaternion)(void *user_data);
	void (*set_quaternion)(void *user_data, VECTOR4D_T value);
	VECTOR4D_T (*get_compass)(void *user_data);
	float (*get_temperature)(void *user_data);
	float (*get_north)(void *user_data);
	void *user_data;
} MPU_T;

typedef struct _MPU_FACTORY_T {
	char name[64];
	void (*release)(void *user_data);
	void (*create_mpu)(void *user_data, MPU_T **mpu);
	void *user_data;
} MPU_FACTORY_T;

typedef struct _DECODER_T {
	char name[64];
	void (*init)(void *user_data, int cam_num, void *display, void *context, void *cam_texture, void **egl_images, int egl_image_num);
	float (*get_fps)(void *user_data);
	int (*get_frameskip)(void *user_data);
	void (*decode)(void *user_data, unsigned char *data, int data_len);
	void (*switch_buffer)(void *user_data);
	void (*release)(void *user_data);
	void *user_data;
} DECODER_T;

typedef struct _DECODER_FACTORY_T {
	char name[64];
	void (*release)(void *user_data);
	void (*create_decoder)(void *user_data, DECODER_T **decoder);
	void *user_data;
} DECODER_FACTORY_T;

typedef struct _RENDERER_T {
	char name[64];
	void (*init)(void *user_data, const char *common, int num_of_cam);
	int (*get_program)(void *user_data);
	void (*render)(void *user_data, float fov);
	void (*release)(void *user_data);
	void *user_data;
} RENDERER_T;

typedef void (*ENCODER_STREAM_CALLBACK)(unsigned char *data, unsigned int data_len, void *frame_data, void *user_data);
typedef struct _ENCODER_T {
	char name[64];
	void (*init)(void *user_data, const int width, const int height, int bitrate_kbps, int fps, ENCODER_STREAM_CALLBACK callback, void *user_data2);
	void (*release)(void *user_data);
	void (*add_frame)(void *user_data, const unsigned char *in_data, void *frame_data);
	void *user_data;
} ENCODER_T;

typedef struct _ENCODER_FACTORY_T {
	char name[64];
	void (*release)(void *user_data);
	void (*create_encoder)(void *user_data, ENCODER_T **encoder);
	void *user_data;
} ENCODER_FACTORY_T;

typedef struct _STATUS_T {
	char name[64];
	void (*get_value)(void *user_data, char *buff, int buff_len);
	void (*set_value)(void *user_data, const char *value);
	void (*release)(void *user_data);
	void *user_data;
} STATUS_T;

typedef struct _PLUGIN_T {
	char name[64];
	void (*release)(void *user_data);
	int (*command_handler)(void *user_data, const char *cmd);
	void (*event_handler)(void *user_data, uint32_t node_id, uint32_t event_id);
	void (*init_options)(void *user_data, json_t *options);
	void (*save_options)(void *user_data, json_t *options);
	char *(*get_info)(void *user_data);
	void *user_data;
	uint32_t node_id;
} PLUGIN_T;

typedef struct _PLUGIN_HOST_T {
	VECTOR4D_T (*get_view_quaternion)();
	VECTOR4D_T (*get_view_compass)();
	float (*get_view_temperature)();
	float (*get_view_north)();

	VECTOR4D_T (*get_camera_offset)(int cam_num);
	void (*set_camera_offset)(int cam_num, VECTOR4D_T value);
	VECTOR4D_T (*get_camera_quaternion)(int cam_num);
	void (*set_camera_quaternion)(int cam_num, VECTOR4D_T value);
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
	void (*set_texture_size)(uint32_t width, uint32_t height);
	int (*load_texture)(const char *filename, uint32_t *tex_out);

	MENU_T *(*get_menu)();
	bool (*get_menu_visible)();
	void (*set_menu_visible)(bool value);

	float (*get_fov)();
	void (*set_fov)(float value);

	void (*send_command)(const char *cmd);
	void (*send_event)(uint32_t node_id, uint32_t event_id);
	void (*add_mpu_factory)(MPU_FACTORY_T *factory);
	void (*add_decoder_factory)(DECODER_FACTORY_T *factory);
	void (*add_encoder_factory)(ENCODER_FACTORY_T *factory);
	void (*add_renderer)(RENDERER_T *renderer);
	void (*add_status)(STATUS_T *status);
	void (*add_watch)(STATUS_T *status);
	void (*add_plugin)(PLUGIN_T *plugin);

	void (*snap)(uint32_t width, uint32_t height, enum RENDERING_MODE mode, const char *path);
} PLUGIN_HOST_T;

typedef void (*CREATE_PLUGIN)(PLUGIN_HOST_T *plugin_host, PLUGIN_T **plugin);

