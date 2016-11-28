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

#define MAX_CAM_NUM 2
#define MAX_OPERATION_NUM 4

enum OPERATION_MODE {
	WINDOW, EQUIRECTANGULAR, FISHEYE, CALIBRATION
};
enum CODEC_TYPE {
	H264, MJPEG
};
typedef struct {
	bool preview;
	bool stereo;
	bool video_direct;
	enum CODEC_TYPE codec_type;
	enum OPERATION_MODE operation_mode;
	uint32_t screen_width;
	uint32_t screen_height;
	uint32_t render_width;
	uint32_t render_height;
	uint32_t cam_width;
	uint32_t cam_height;
	uint32_t active_cam;
// OpenGL|ES objects
	EGLDisplay display;
	EGLSurface surface;
	EGLContext context;
	struct {
		void *render;
		void *render_ary[MAX_OPERATION_NUM];
		void *stereo;
	} program;
	GLuint render_vbo;
	GLuint render_vbo_nop;
	GLuint render_vbo_ary[MAX_OPERATION_NUM];
	GLuint render_vbo_nop_ary[MAX_OPERATION_NUM];
	float render_vbo_scale;
	GLuint stereo_vbo;
	GLuint stereo_vbo_nop;
	int num_of_cam;
	GLuint cam_texture[MAX_CAM_NUM];
	GLuint logo_texture;
	GLuint render_texture;
	GLuint framebuffer;
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

	bool recording;
	bool snap;
	char snap_save_path[256];
} CUBE_STATE_T;
