/*
 Copyright (c) 2012, Broadcom Europe Ltd
 Copyright (c) 2012, OtherCrashOverride
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

// A rotating cube rendered with OpenGL|ES. Three images used as textures on the cube faces.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <stdbool.h>
#include <linux/input.h>

#include "bcm_host.h"

#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"

#include "picam360_capture.h"
#include "video.h"
#include "video_mjpeg.h"
#include "video_direct.h"
#include "picam360_tools.h"
#include "gl_program.h"
#include "device.h"

#include <mat4/type.h>
#include <mat4/create.h>
#include <mat4/identity.h>
#include <mat4/rotateX.h>
#include <mat4/rotateY.h>
#include <mat4/rotateZ.h>
#include <mat4/scale.h>
#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/perspective.h>

//json parser
#include <jansson.h>

#include <opencv/highgui.h>

#define PATH "./"

#ifndef M_PI
#define M_PI 3.141592654
#endif

#define CONFIG_FILE "config.json"

typedef struct {
	float sharpness_gain;
	float cam_offset_roll[MAX_CAM_NUM];
	float cam_offset_pitch[MAX_CAM_NUM];
	float cam_offset_yaw[MAX_CAM_NUM];
	float cam_offset_x[MAX_CAM_NUM];
	float cam_offset_y[MAX_CAM_NUM];
	float cam_horizon_r[MAX_CAM_NUM];
} OPTIONS_T;
OPTIONS_T lg_options = { };

static void init_ogl(PICAM360CAPTURE_T *state);
static void init_model_proj(PICAM360CAPTURE_T *state);
static void init_textures(PICAM360CAPTURE_T *state);
static void init_options(PICAM360CAPTURE_T *state);
static void save_options(PICAM360CAPTURE_T *state);
static void exit_func(void);
static void redraw_render_texture(PICAM360CAPTURE_T *state, FRAME_T *frame,
		MODEL_T *model);
static void redraw_scene(PICAM360CAPTURE_T *state, FRAME_T *frame,
		MODEL_T *model);

static volatile int terminate;
static PICAM360CAPTURE_T _state, *state = &_state;

/***********************************************************
 * Name: init_ogl
 *
 * Arguments:
 *       PICAM360CAPTURE_T *state - holds OGLES model info
 *
 * Description: Sets the display, OpenGL|ES context and screen stuff
 *
 * Returns: void
 *
 ***********************************************************/
static void init_ogl(PICAM360CAPTURE_T *state) {
	int32_t success = 0;
	EGLBoolean result;
	EGLint num_config;

	static EGL_DISPMANX_WINDOW_T nativewindow;

	DISPMANX_ELEMENT_HANDLE_T dispman_element;
	DISPMANX_DISPLAY_HANDLE_T dispman_display;
	DISPMANX_UPDATE_HANDLE_T dispman_update;
	VC_RECT_T dst_rect;
	VC_RECT_T src_rect;

	static const EGLint attribute_list[] = { EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8,
			EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_DEPTH_SIZE, 16,
			//EGL_SAMPLES, 4,
			EGL_SURFACE_TYPE, EGL_WINDOW_BIT, EGL_NONE };
	static const EGLint context_attributes[] = { EGL_CONTEXT_CLIENT_VERSION, 2,
			EGL_NONE };

	EGLConfig config;

	// get an EGL display connection
	state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	assert(state->display != EGL_NO_DISPLAY);

	// initialize the EGL display connection
	result = eglInitialize(state->display, NULL, NULL);
	assert(EGL_FALSE != result);

	// get an appropriate EGL frame buffer configuration
	// this uses a BRCM extension that gets the closest match, rather than standard which returns anything that matches
	result = eglSaneChooseConfigBRCM(state->display, attribute_list, &config, 1,
			&num_config);
	assert(EGL_FALSE != result);

	//Bind to the right EGL API.
	result = eglBindAPI(EGL_OPENGL_ES_API);
	assert(result != EGL_FALSE);

	// create an EGL rendering context
	state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT,
			context_attributes);
	assert(state->context != EGL_NO_CONTEXT);

	// create an EGL window surface
	success = graphics_get_display_size(0 /* LCD */, &state->screen_width,
			&state->screen_height);
	assert(success >= 0);

	dst_rect.x = 0;
	dst_rect.y = 0;
	dst_rect.width = state->screen_width;
	dst_rect.height = state->screen_height;

	src_rect.x = 0;
	src_rect.y = 0;
	src_rect.width = state->screen_width << 16;
	src_rect.height = state->screen_height << 16;

	//if (state->preview)
	{
		dispman_display = vc_dispmanx_display_open(0 /* LCD */);
		dispman_update = vc_dispmanx_update_start(0);

		dispman_element = vc_dispmanx_element_add(dispman_update,
				dispman_display, 0/*layer*/, &dst_rect, 0/*src*/, &src_rect,
				DISPMANX_PROTECTION_NONE, 0 /*alpha*/, 0/*clamp*/,
				0/*transform*/);

		nativewindow.element = dispman_element;
		nativewindow.width = state->screen_width;
		nativewindow.height = state->screen_height;
		vc_dispmanx_update_submit_sync(dispman_update);

		state->surface = eglCreateWindowSurface(state->display, config,
				&nativewindow, NULL);
	}
//	else {
//		//Create an offscreen rendering surface
//		EGLint rendering_attributes[] = { EGL_WIDTH, state->screen_width,
//				EGL_HEIGHT, state->screen_height, EGL_NONE };
//		state->surface = eglCreatePbufferSurface(state->display, config,
//				rendering_attributes);
//	}
	assert(state->surface != EGL_NO_SURFACE);

	// connect the context to the surface
	result = eglMakeCurrent(state->display, state->surface, state->surface,
			state->context);
	assert(EGL_FALSE != result);

	// Enable back face culling.
	glEnable(GL_CULL_FACE);
}

int load_texture(const char *filename, GLuint *tex_out) {
	GLuint tex;
	IplImage *iplImage = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR);

	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, iplImage->width, iplImage->height, 0,
			GL_RGB, GL_UNSIGNED_BYTE, iplImage->imageData);
	if (glGetError() != GL_NO_ERROR) {
		printf("glTexImage2D failed. Could not allocate texture buffer.");
	}
	glBindTexture(GL_TEXTURE_2D, 0);
	if (tex_out != NULL)
		*tex_out = tex;

	return 0;
}

int board_mesh(GLuint *vbo_out, GLuint *n_out) {
	GLuint vbo;
	static const GLfloat quad_vertex_positions[] = { 0.0f, 0.0f, 1.0f, 1.0f,
			1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
			1.0f };

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertex_positions),
			quad_vertex_positions, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	if (vbo_out != NULL)
		*vbo_out = vbo;
	if (n_out != NULL)
		*n_out = 4;

	return 0;
}

int spherewindow_mesh(float theta_degree, int phi_degree, int num_of_steps,
		GLuint *vbo_out, GLuint *n_out, float *scale_out) {
	GLuint vbo;

	int n = 2 * (num_of_steps + 1) * num_of_steps;
	float points[4 * n];

	float theta = theta_degree * M_PI / 180.0;
	float phi = phi_degree * M_PI / 180.0;

	float start_x = -tan(theta / 2);
	float start_y = -tan(phi / 2);

	float end_x = tan(theta / 2);
	float end_y = tan(phi / 2);

	float step_x = (end_x - start_x) / num_of_steps;
	float step_y = (end_y - start_y) / num_of_steps;

	float scale = 0;
	int idx = 0;
	int i, j;
	for (i = 0; i < num_of_steps; i++) {	//x
		for (j = 0; j <= num_of_steps; j++) {	//y
			{
				float x = start_x + step_x * i;
				float y = start_y + step_y * j;
				float z = 1.0;
				float len = sqrt(x * x + y * y + z * z);
				points[idx++] = x / len;
				points[idx++] = y / len;
				points[idx++] = z / len;
				points[idx++] = 1.0;
				if (scale == 0) {
					scale = z / -x;
				}
				//printf("x=%f,y=%f,z=%f,w=%f\n", points[idx - 4],
				//		points[idx - 3], points[idx - 2], points[idx - 1]);
			}
			{
				float x = start_x + step_x * (i + 1);
				float y = start_y + step_y * j;
				float z = 1.0;
				float len = sqrt(x * x + y * y + z * z);
				points[idx++] = x / len;
				points[idx++] = y / len;
				points[idx++] = z / len;
				points[idx++] = 1.0;
				//printf("x=%f,y=%f,z=%f,w=%f\n", points[idx - 4],
				//		points[idx - 3], points[idx - 2], points[idx - 1]);
			}
		}
	}

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * n, points,
			GL_STATIC_DRAW);

	if (vbo_out != NULL)
		*vbo_out = vbo;
	if (n_out != NULL)
		*n_out = n;
	if (scale_out != NULL)
		*scale_out = scale;

	return 0;
}

/***********************************************************
 * Name: init_model_proj
 *
 * Arguments:
 *       PICAM360CAPTURE_T *state - holds OGLES model info
 *
 * Description: Sets the OpenGL|ES model to default values
 *
 * Returns: void
 *
 ***********************************************************/
static void init_model_proj(PICAM360CAPTURE_T *state, MODEL_T *model_data) {
	float fov = 120.0;
	float aspect = state->render_height / state->render_width;

	board_mesh(&model_data[EQUIRECTANGULAR].vbo,
			&model_data[EQUIRECTANGULAR].vbo_nop);
	if (state->num_of_cam == 1) {
		model_data[EQUIRECTANGULAR].program = GLProgram_new(
				"shader/equirectangular.vert", "shader/equirectangular.frag");
	} else {
		model_data[EQUIRECTANGULAR].program = GLProgram_new(
				"shader/equirectangular.vert",
				"shader/equirectangular_sphere.frag");
	}

	board_mesh(&model_data[FISHEYE].vbo, &model_data[FISHEYE].vbo_nop);
	model_data[FISHEYE].program = GLProgram_new("shader/fisheye.vert",
			"shader/fisheye.frag");

	board_mesh(&model_data[CALIBRATION].vbo, &model_data[CALIBRATION].vbo_nop);
	model_data[CALIBRATION].program = GLProgram_new("shader/calibration.vert",
			"shader/calibration.frag");

	spherewindow_mesh(fov, fov * aspect, 50, &model_data[WINDOW].vbo,
			&model_data[WINDOW].vbo_nop, &model_data[WINDOW].scale);
	if (state->num_of_cam == 1) {
		model_data[WINDOW].program = GLProgram_new("shader/window.vert",
				"shader/window.frag");
	} else {
		model_data[WINDOW].program = GLProgram_new("shader/window.vert",
				"shader/window_sphere.frag");
	}

	board_mesh(&model_data[BOARD].vbo, &model_data[BOARD].vbo_nop);
	model_data[BOARD].program = GLProgram_new("shader/board.vert",
			"shader/board.frag");
}

/***********************************************************
 * Name: init_textures
 *
 * Arguments:
 *       PICAM360CAPTURE_T *state - holds OGLES model info
 *
 * Description:   Initialise OGL|ES texture surfaces to use image
 *                buffers
 *
 * Returns: void
 *
 ***********************************************************/
static void init_textures(PICAM360CAPTURE_T *state) {

	load_texture("img/calibration_img.png", &state->calibration_texture);
	load_texture("img/logo_img.png", &state->logo_texture);

	for (int i = 0; i < state->num_of_cam; i++) {
		//// load three texture buffers but use them on six OGL|ES texture surfaces
		glGenTextures(1, &state->cam_texture[i]);

		glBindTexture(GL_TEXTURE_2D, state->cam_texture[i]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, state->cam_width,
				state->cam_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		/* Create EGL Image */
		state->egl_image[i] = eglCreateImageKHR(state->display, state->context,
				EGL_GL_TEXTURE_2D_KHR, (EGLClientBuffer) state->cam_texture[i],
				0);

		if (state->egl_image[i] == EGL_NO_IMAGE_KHR) {
			printf("eglCreateImageKHR failed.\n");
			exit(1);
		}

		// Start rendering
		void **args = malloc(sizeof(void*) * 3);
		args[0] = (void*) i;
		args[1] = (void*) state->egl_image[i];
		args[2] = (void*) state;
		pthread_create(&state->thread[i], NULL,
				(state->video_direct) ? video_direct :
				(state->codec_type == H264) ?
						video_decode_test : video_mjpeg_decode, args);

		glEnable(GL_TEXTURE_2D);

		// Bind texture surface to current vertices
		glBindTexture(GL_TEXTURE_2D, state->cam_texture[i]);
	}
}
//------------------------------------------------------------------------------

/***********************************************************
 * Name: init_options
 *
 * Arguments:
 *       PICAM360CAPTURE_T *state - holds OGLES model info
 *
 * Description:   Initialise options
 *
 * Returns: void
 *
 ***********************************************************/
static void init_options(PICAM360CAPTURE_T *state) {
	json_error_t error;
	json_t *options = json_load_file(CONFIG_FILE, 0, &error);
	if (options == NULL) {
		fputs(error.text, stderr);
	} else {
		lg_options.sharpness_gain = json_number_value(
				json_object_get(options, "sharpness_gain"));
		for (int i = 0; i < MAX_CAM_NUM; i++) {
			char buff[256];
			sprintf(buff, "cam%d_offset_roll", i);
			lg_options.cam_offset_roll[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_offset_pitch", i);
			lg_options.cam_offset_pitch[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_offset_yaw", i);
			lg_options.cam_offset_yaw[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_offset_x", i);
			lg_options.cam_offset_x[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_offset_y", i);
			lg_options.cam_offset_y[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_horizon_r", i);
			lg_options.cam_horizon_r[i] = json_number_value(
					json_object_get(options, buff));

			if (lg_options.cam_horizon_r[i] == 0) {
				lg_options.cam_horizon_r[i] = 0.8;
			}
		}

		json_decref(options);
	}
}
//------------------------------------------------------------------------------

/***********************************************************
 * Name: save_options
 *
 * Arguments:
 *       PICAM360CAPTURE_T *state - holds OGLES model info
 *
 * Description:   Initialise options
 *
 * Returns: void
 *
 ***********************************************************/
static void save_options(PICAM360CAPTURE_T *state) {
	json_t *options = json_object();

	json_object_set_new(options, "sharpness_gain",
			json_real(lg_options.sharpness_gain));
	for (int i = 0; i < MAX_CAM_NUM; i++) {
		char buff[256];
		sprintf(buff, "cam%d_offset_roll", i);
		json_object_set_new(options, buff,
				json_real(lg_options.cam_offset_roll[i]));
		sprintf(buff, "cam%d_offset_pitch", i);
		json_object_set_new(options, buff,
				json_real(lg_options.cam_offset_pitch[i]));
		sprintf(buff, "cam%d_offset_yaw", i);
		json_object_set_new(options, buff,
				json_real(lg_options.cam_offset_yaw[i]));
		sprintf(buff, "cam%d_offset_x", i);
		json_object_set_new(options, buff,
				json_real(lg_options.cam_offset_x[i]));
		sprintf(buff, "cam%d_offset_y", i);
		json_object_set_new(options, buff,
				json_real(lg_options.cam_offset_y[i]));
		sprintf(buff, "cam%d_horizon_r", i);
		json_object_set_new(options, buff,
				json_real(lg_options.cam_horizon_r[i]));
	}

	json_dump_file(options, CONFIG_FILE, 0);

	json_decref(options);
}
//------------------------------------------------------------------------------

static void exit_func(void)
// Function to be passed to atexit().
{
	for (int i = 0; i < state->num_of_cam; i++) {
		if (state->egl_image[i] != 0) {
			if (!eglDestroyImageKHR(state->display,
					(egl_imageKHR) state->egl_image[i]))
				printf("eglDestroyImageKHR failed.");
			state->egl_image[i] = NULL;
		}
	}

	// clear screen
	glClear(GL_COLOR_BUFFER_BIT);
	eglSwapBuffers(state->display, state->surface);

	// Release OpenGL resources
	eglMakeCurrent(state->display, EGL_NO_SURFACE, EGL_NO_SURFACE,
			EGL_NO_CONTEXT);
	eglDestroySurface(state->display, state->surface);
	eglDestroyContext(state->display, state->context);
	eglTerminate(state->display);

	printf("\npicam360-capture closed\n");
} // exit_func()

//==============================================================================

bool inputAvailable() {
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
	return (FD_ISSET(0, &fds));
}

bool setRenderSize(PICAM360CAPTURE_T *state, FRAME_T *frame, int render_width,
		int render_height) {
	if (render_width >= 2048 || render_height >= 1024) {
		return false;
	} else {
		frame->width = render_width;
		frame->height = render_height;
	}

	printf("width=%d,height=%d\n", frame->width, frame->height);

	if (frame->framebuffer) {
		glDeleteFramebuffers(1, &frame->framebuffer);
	}

	//texture rendering
	glGenFramebuffers(1, &frame->framebuffer);

	if (frame->texture) {
		glDeleteTextures(1, &frame->texture);
	}

	glGenTextures(1, &frame->exture);
	glBindTexture(GL_TEXTURE_2D, frame->texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame->width,
			frame->height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	if (glGetError() != GL_NO_ERROR) {
		printf("glTexImage2D failed. Could not allocate texture buffer.\n");
	}
	glBindTexture(GL_TEXTURE_2D, 0);

	glBindFramebuffer(GL_FRAMEBUFFER, frame->framebuffer);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
			frame->texture, 0);
	if (glGetError() != GL_NO_ERROR) {
		printf(
				"glFramebufferTexture2D failed. Could not allocate framebuffer.\n");
	}

	// Set background color and clear buffers
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return true;
}

int main(int argc, char *argv[]) {
	MODEL_T model_data[MAX_OPERATION_NUM];
	bool res;
	int opt;
	int render_width = 512;
	int render_height = 512;
	// Clear application state
	memset(state, 0, sizeof(*state));
	state->cam_width = 1024;
	state->cam_height = 1024;
	state->num_of_cam = 1;
	state->preview = false;
	state->stereo = false;
	state->codec_type = H264;
	state->operation_mode = WINDOW;
	state->video_direct = false;

	//init options
	init_options(state);

	while ((opt = getopt(argc, argv, "c:w:h:n:psW:H:ECFD")) != -1) {
		switch (opt) {
		case 'c':
			if (strcmp(optarg, "MJPEG") == 0) {
				state->codec_type = MJPEG;
			}
			break;
		case 'w':
			sscanf(optarg, "%d", &state->cam_width);
			break;
		case 'h':
			sscanf(optarg, "%d", &state->cam_height);
			break;
		case 'n':
			sscanf(optarg, "%d", &state->num_of_cam);
			break;
		case 'p':
			state->preview = true;
			break;
		case 's':
			state->stereo = true;
			break;
		case 'W':
			sscanf(optarg, "%d", &render_width);
			break;
		case 'H':
			sscanf(optarg, "%d", &render_height);
			break;
		case 'E':
			state->operation_mode = EQUIRECTANGULAR;
			break;
		case 'C':
			state->operation_mode = CALIBRATION;
			break;
		case 'F':
			state->operation_mode = FISHEYE;
			break;
		case 'D':
			state->video_direct = true;
			break;
		default:
			/* '?' */
			printf(
					"Usage: %s [-w width] [-h height] [-n num_of_cam] [-p] [-s]\n",
					argv[0]);
			return -1;
		}
	}

	bcm_host_init();
	printf("Note: ensure you have sufficient gpu_mem configured\n");

	init_device();

	// Start OGLES
	init_ogl(state);

	//set render size. this should be after init_ogl()
	res = setRenderSize(state, render_width, render_height);
	if (!res) {
		printf("render size error");
		exit(-1);
	}

	// Setup the model world
	init_model_proj(state);

	// initialise the OGLES texture(s)
	init_textures(state);

	double calib_step = 0.01;
	int frame_num;
	double frame_elapsed;
	struct timeval s, f;
	double elapsed_ms;
	int size = state->render_width * state->render_height * 3;
	unsigned char *image_buffer = (unsigned char*) malloc(size);
	unsigned char *image_buffer_double = (unsigned char*) malloc(size * 2);

	while (!terminate) {
		if (inputAvailable()) {
			char buff[256];
			int size = read(STDIN_FILENO, buff, sizeof(buff) - 1);
			buff[size] = '\0';
			char *cmd = strtok(buff, " \n");
			if (cmd == NULL) {
				//do nothing
			} else if (strncmp(cmd, "exit", sizeof(buff)) == 0) {
				printf("exit\n");
				exit(0); //temporary
				break;
			} else if (strncmp(cmd, "0", sizeof(buff)) == 0) {
				state->active_cam = 0;
			} else if (strncmp(cmd, "1", sizeof(buff)) == 0) {
				state->active_cam = 1;
			} else if (strncmp(cmd, "snap", sizeof(buff)) == 0) {
				char *param = strtok(NULL, " \n");
				if (param != NULL) {
					strncpy(state->snap_save_path, param,
							sizeof(state->snap_save_path) - 1);
					state->snap = true;
				}
			} else if (strncmp(cmd, "start_record", sizeof(buff)) == 0) {
				char *param = strtok(NULL, " \n");
				if (param != NULL) {
					StartRecord(state->render_width, state->render_height,
							param, 4000);
					state->recording = true;
					frame_num = 0;
					frame_elapsed = 0;
					printf("start_record saved to %s\n", param);
				}
			} else if (strncmp(cmd, "stop_record", sizeof(buff)) == 0) {
				printf("stop_record\n");
				if (state->recording) {
					state->recording = false;
					StopRecord();

					frame_elapsed /= frame_num;
					printf("stop record : frame num : %d : fps %.3lf\n",
							frame_num, 1000.0 / frame_elapsed);
				}
			} else if (strncmp(cmd, "set_mode", sizeof(buff)) == 0) {
				char *param = strtok(NULL, " \n");
				if (param != NULL) {
					switch (param[0]) {
					case 'W':
						state->operation_mode = WINDOW;
						break;
					case 'E':
						state->operation_mode = EQUIRECTANGULAR;
						break;
					case 'F':
						state->operation_mode = FISHEYE;
						break;
					case 'C':
						state->operation_mode = CALIBRATION;
						break;
					default:
						printf("unknown mode %s\n", param);
					}
					printf("set_mode %s\n", param);
				}
			} else if (strncmp(cmd, "set_camera_orientation", sizeof(buff))
					== 0) {
				char *param = strtok(NULL, " \n");
				if (param != NULL) {
					float roll;
					float pitch;
					float yaw;
					sscanf(param, "%f,%f,%f", &roll, &pitch, &yaw);
					state->camera_roll = roll * M_PI / 180.0;
					state->camera_pitch = pitch * M_PI / 180.0;
					state->camera_yaw = yaw * M_PI / 180.0;
					printf("set_camera_orientation\n");
				}
			} else if (strncmp(cmd, "set_render_size", sizeof(buff)) == 0) {
				char *param = strtok(NULL, " \n");
				if (param != NULL) {
					int render_width;
					int render_height;
					sscanf(param, "%d,%d", &render_width, &render_height);
					res = setRenderSize(state, render_width, render_height);
					if (!res) {
						printf("error in %s\n", param);
					}
					printf("set_render_size %s\n", param);
				}
			} else if (strncmp(cmd, "set_stereo", sizeof(buff)) == 0) {
				char *param = strtok(NULL, " \n");
				if (param != NULL) {
					state->stereo = (param[0] == '1');
					printf("set_stereo %s\n", param);
				}
			} else if (strncmp(cmd, "set_preview", sizeof(buff)) == 0) {
				char *param = strtok(NULL, " \n");
				if (param != NULL) {
					state->preview = (param[0] == '1');
					printf("set_preview %s\n", param);
				}
			} else if (state->operation_mode == CALIBRATION) {
				if (strncmp(cmd, "step", sizeof(buff)) == 0) {
					char *param = strtok(NULL, " \n");
					if (param != NULL) {
						sscanf(param, "%lf", &calib_step);
					}
				}
				if (strncmp(cmd, "u", sizeof(buff)) == 0) {
					lg_options.cam_offset_y[state->active_cam] -= calib_step;
				}
				if (strncmp(cmd, "d", sizeof(buff)) == 0) {
					lg_options.cam_offset_y[state->active_cam] += calib_step;
				}
				if (strncmp(cmd, "l", sizeof(buff)) == 0) {
					lg_options.cam_offset_x[state->active_cam] += calib_step;
				}
				if (strncmp(cmd, "r", sizeof(buff)) == 0) {
					lg_options.cam_offset_x[state->active_cam] -= calib_step;
				}
				if (strncmp(cmd, "s", sizeof(buff)) == 0) {
					lg_options.sharpness_gain += calib_step;
				}
				if (strncmp(cmd, "w", sizeof(buff)) == 0) {
					lg_options.sharpness_gain -= calib_step;
				}
				if (strncmp(cmd, "save", sizeof(buff)) == 0) {
					save_options(state);
				}
			} else {
				printf("unknown command : %s\n", buff);
			}
		}
		gettimeofday(&s, NULL);
		if (state->preview) {
			redraw_render_texture(state);
			redraw_scene(state);
		}
		if (state->recording || state->snap) {
			int img_width;
			int img_height;
			unsigned char *img_buff;

			if (state->double_size) {
				img_width = state->render_width * 2;
				img_height = state->render_height * 2;
				img_buff = image_buffer_double;
				for (int split = 0; split < 2; split++) {
					state->split = split + 1;
					redraw_render_texture(state);
					glFinish();
					glBindFramebuffer(GL_FRAMEBUFFER, state->framebuffer);
					glReadPixels(0, 0, state->render_width,
							state->render_height, GL_RGB, GL_UNSIGNED_BYTE,
							image_buffer);
					glBindFramebuffer(GL_FRAMEBUFFER, 0);
					for (int y = 0; y < state->render_height; y++) {
						memcpy(
								image_buffer_double
										+ state->render_width * 2 * 3 * y
										+ state->render_width * 3 * split,
								image_buffer + state->render_width * 3 * y,
								state->render_width * 3);
					}
				}
			} else {
				img_width = state->render_width;
				img_height = state->render_height;
				img_buff = image_buffer;
				if (!state->preview) {
					redraw_render_texture(state);
					glFinish();
				}
				glBindFramebuffer(GL_FRAMEBUFFER, state->framebuffer);
				glReadPixels(0, 0, state->render_width, state->render_height,
						GL_RGB, GL_UNSIGNED_BYTE, image_buffer);
				glBindFramebuffer(GL_FRAMEBUFFER, 0);
			}

			if (state->recording) {
				AddFrame(img_buff);

				gettimeofday(&f, NULL);
				elapsed_ms = (f.tv_sec - s.tv_sec) * 1000.0
						+ (f.tv_usec - s.tv_usec) / 1000.0;
				frame_num++;
				frame_elapsed += elapsed_ms;
			}
			if (state->snap) {
				state->snap = false;
				SaveJpeg(img_buff, img_width, img_height, state->snap_save_path,
						70);
				printf("snap saved to %s\n", state->snap_save_path);

				gettimeofday(&f, NULL);
				elapsed_ms = (f.tv_sec - s.tv_sec) * 1000.0
						+ (f.tv_usec - s.tv_usec) / 1000.0;
				printf("elapsed %.3lf ms\n", elapsed_ms);
			}
		}
		gettimeofday(&f, NULL);
	}
	exit_func();
	return 0;
}

/***********************************************************
 * Name: redraw_scene
 *
 * Arguments:
 *       PICAM360CAPTURE_T *state - holds OGLES model info
 *
 * Description:   Draws the model and calls eglSwapBuffers
 *                to render to screen
 *
 * Returns: void
 *
 ***********************************************************/
static void redraw_render_texture(PICAM360CAPTURE_T *state, FRAME_T *frame,
		MODEL_T *model) {
	int program = GLProgram_GetId(model->program);
	glUseProgram(program);

	glBindFramebuffer(GL_FRAMEBUFFER, frame->framebuffer);

	glViewport(0, 0, frame->width, frame->height);

	glBindBuffer(GL_ARRAY_BUFFER, model->vbo);
	glActiveTexture(GL_TEXTURE0);
	if (state->operation_mode == CALIBRATION) {
		glBindTexture(GL_TEXTURE_2D, state->calibration_texture);
	} else {
		glBindTexture(GL_TEXTURE_2D, state->logo_texture);
	}
	for (int i = 0; i < state->num_of_cam; i++) {
		glActiveTexture(GL_TEXTURE1 + i);
		glBindTexture(GL_TEXTURE_2D, state->cam_texture[i]);
	}

	mat4 camera_matrix = mat4_create();
	mat4_identity(camera_matrix);
	mat4_rotateY(camera_matrix, camera_matrix, state->camera_yaw); //vertical asis is y
	mat4_rotateZ(camera_matrix, camera_matrix, state->camera_pitch); //depth axis is z
	mat4_rotateX(camera_matrix, camera_matrix,
			state->camera_roll + lg_options.cam_offset_roll[0]);

	mat4 unif_matrix = mat4_create();
	mat4_fromQuat(unif_matrix, get_quatanion());
	mat4_rotateX(unif_matrix, unif_matrix, -M_PI / 2 + M_PI); //M_PI for jpeg coordinate
	//float scale_factor[3] = { 1.0, 1.0, -1.0 };
	//mat4_scale(unif_matrix, unif_matrix, scale_factor);

	mat4_multiply(unif_matrix, camera_matrix, unif_matrix);

	//Load in the texture and thresholding parameters.
	glUniform1f(glGetUniformLocation(program, "split"), state->split);
	glUniform1f(glGetUniformLocation(program, "scale"), model->scale);
	glUniform1f(glGetUniformLocation(program, "pixel_size"),
			1.0 / state->cam_width);

	glUniform1i(glGetUniformLocation(program, "active_cam"), state->active_cam);

	//options start
	glUniform1f(glGetUniformLocation(program, "sharpness_gain"),
			lg_options.sharpness_gain);
	for (int i = 0; i < state->num_of_cam; i++) {
		char buff[256];
		sprintf(buff, "cam%d_offset_yaw", i);
		glUniform1f(glGetUniformLocation(program, buff),
				lg_options.cam_offset_yaw[i]);
		sprintf(buff, "cam%d_offset_x", i);
		glUniform1f(glGetUniformLocation(program, buff),
				lg_options.cam_offset_x[i]);
		sprintf(buff, "cam%d_offset_y", i);
		glUniform1f(glGetUniformLocation(program, buff),
				lg_options.cam_offset_y[i]);
		sprintf(buff, "cam%d_horizon_r", i);
		glUniform1f(glGetUniformLocation(program, buff),
				lg_options.cam_horizon_r[i]);
	}
	glUniform1f(glGetUniformLocation(program, "cam_offset_yaw"),
			lg_options.cam_offset_yaw[state->active_cam]);
	glUniform1f(glGetUniformLocation(program, "cam_offset_x"),
			lg_options.cam_offset_x[state->active_cam]);
	glUniform1f(glGetUniformLocation(program, "cam_offset_y"),
			lg_options.cam_offset_y[state->active_cam]);
	glUniform1f(glGetUniformLocation(program, "cam_horizon_r"),
			lg_options.cam_horizon_r[state->active_cam]);
	//options end

	//texture start
	glUniform1i(glGetUniformLocation(program, "logo_texture"), 0);
	for (int i = 0; i < state->num_of_cam; i++) {
		char buff[256];
		sprintf(buff, "cam%d_texture", i);
		glUniform1i(glGetUniformLocation(program, buff), i + 1);
	}
	glUniform1i(glGetUniformLocation(program, "cam_texture"),
			state->active_cam + 1);
	//texture end

	glUniformMatrix4fv(glGetUniformLocation(program, "unif_matrix"), 1,
			GL_FALSE, (GLfloat*) unif_matrix);

	GLuint loc = glGetAttribLocation(program, "vPosition");
	glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(loc);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, model->vbo_nop);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

static void redraw_scene(PICAM360CAPTURE_T *state, FRAME_T *frame,
		MODEL_T *model) {
	int program = GLProgram_GetId(model->program);
	glUseProgram(program);

	// Start with a clear screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBindBuffer(GL_ARRAY_BUFFER, model->vbo);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, frame->texture);

	//Load in the texture and thresholding parameters.
	glUniform1i(glGetUniformLocation(program, "tex"), 0);

	GLuint loc = glGetAttribLocation(program, "vPosition");
	glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(loc);

	if (state->operation_mode == CALIBRATION) {
		glViewport((state->screen_width - state->screen_height) / 2, 0,
				(GLsizei) state->screen_height, (GLsizei) state->screen_height);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, model->vbo_nop);
	} else if (state->stereo) {
		int offset_x = (state->screen_width / 2 - frame->width) / 2;
		int offset_y = (state->screen_height - frame->height) / 2;
		for (int i = 0; i < 2; i++) {
			//glViewport(0, 0, (GLsizei)state->screen_width/2, (GLsizei)state->screen_height);
			glViewport(offset_x + i * state->screen_width / 2, offset_y,
					(GLsizei) frame->width, (GLsizei) frame->height);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, model->vbo_nop);
		}
	} else {
		int offset_x = (state->screen_width - frame->width) / 2;
		int offset_y = (state->screen_height - frame->height) / 2;
		glViewport(offset_x, offset_y, (GLsizei) frame->width,
				(GLsizei) frame->height);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, model->vbo_nop);
	}

	eglSwapBuffers(state->display, state->surface);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
}

