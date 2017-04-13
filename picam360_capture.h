/*
 Copyright (c) 2012, Broadcom Europe Ltd
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <stdbool.h>
#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#include <pthread.h>
#include "mrevent.h"

#include "picam360_capture_plugin.h"

#define MAX_CAM_NUM 2
#define MAX_OPERATION_NUM 5

enum INPUT_MODE {
	INPUT_MODE_NONE, INPUT_MODE_CAM, INPUT_MODE_FILE
};
enum OUTPUT_MODE {
	OUTPUT_MODE_NONE, OUTPUT_MODE_STILL, OUTPUT_MODE_VIDEO
};
enum OPERATION_MODE {
	BOARD, WINDOW, EQUIRECTANGULAR, FISHEYE, CALIBRATION
};
enum CODEC_TYPE {
	H264, MJPEG
};
enum VIEW_COODINATE_MODE {
	MANUAL, MPU9250, OCULUS_RIFT
};

struct _PICAM360CAPTURE_T;

typedef struct _OPTIONS_T {
	float sharpness_gain;
	float cam_offset_pitch[MAX_CAM_NUM]; // x axis
	float cam_offset_yaw[MAX_CAM_NUM]; // y axis
	float cam_offset_roll[MAX_CAM_NUM]; // z axis
	float cam_offset_x[MAX_CAM_NUM];
	float cam_offset_y[MAX_CAM_NUM];
	float cam_horizon_r[MAX_CAM_NUM];

	float view_offset_pitch; // x axis
	float view_offset_yaw; // y axis
	float view_offset_roll; // z axis
} OPTIONS_T;

typedef struct _FRAME_T {
	int id;
	GLuint framebuffer;
	GLuint texture;
	uint8_t *img_buff;
	uint32_t width;
	uint32_t height;
	bool delete_after_processed;
	int frame_num;
	double frame_elapsed;
	bool is_recording;
	void *recorder;

	enum OPERATION_MODE operation_mode;
	enum OUTPUT_MODE output_mode;
	char output_filepath[256];
	bool double_size;

	float fov;
	//for unif matrix
	//euler angles
	float view_pitch;
	float view_yaw;
	float view_roll;
	enum VIEW_COODINATE_MODE view_coordinate_mode;

	void *custom_data;
	//event
	void (*after_processed_callback)(struct _PICAM360CAPTURE_T *,
			struct _FRAME_T *);
	void (*befor_deleted_callback)(struct _PICAM360CAPTURE_T *,
			struct _FRAME_T *);

	struct _FRAME_T *next;
} FRAME_T;
typedef struct {
	void *program;
	GLuint vbo;
	GLuint vbo_nop;
} MODEL_T;
typedef struct _PICAM360CAPTURE_T {
	PLUGIN_HOST_T plugin_host;
	int split;
	bool preview;
	bool stereo;
	bool video_direct;
	enum CODEC_TYPE codec_type;
	uint32_t screen_width;
	uint32_t screen_height;
	uint32_t cam_width;
	uint32_t cam_height;
// OpenGL|ES objects
	EGLDisplay display;
	EGLSurface surface;
	EGLContext context;
	int active_cam;
	int num_of_cam;
	pthread_t thread[MAX_CAM_NUM];
	void* egl_image[MAX_CAM_NUM][2];//double buffer
	GLuint cam_texture[MAX_CAM_NUM][2];//double buffer
	int cam_texture_cur[MAX_CAM_NUM];
	GLuint logo_texture;
	GLuint calibration_texture;
// model rotation vector and direction
	GLfloat rot_angle_x_inc;
	GLfloat rot_angle_y_inc;
	GLfloat rot_angle_z_inc;
// current model rotation angles
	GLfloat rot_angle_x;
	GLfloat rot_angle_y;
	GLfloat rot_angle_z;
// current distance from camera
	GLfloat distance;
	GLfloat distance_inc;

	MREVENT_T request_frame_event[MAX_CAM_NUM];
	MREVENT_T arrived_frame_event[MAX_CAM_NUM];
	enum INPUT_MODE input_mode;
	char input_filepath[256];
	int input_file_size;
	int input_file_cur;
	bool frame_sync;
	bool output_raw;
	char output_raw_filepath[256];

	//for unif matrix
	//euler angles
	float camera_pitch; // x axis
	float camera_yaw; // y axis
	float camera_roll; // z axis
	float camera_quatanion[MAX_CAM_NUM][4];
	float camera_compass[4];
	float camera_temperature;
	float camera_north;
	bool camera_coordinate_from_device;
	enum VIEW_COODINATE_MODE default_view_coordinate_mode;

	FRAME_T *frame;
	MODEL_T model_data[MAX_OPERATION_NUM];
	pthread_mutex_t texture_mutex;

	MENU_T *menu;

	PLUGIN_T **plugins;
	struct _OPTIONS_T options;
} PICAM360CAPTURE_T;
