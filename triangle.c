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

#include "bcm_host.h"

#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"

#include "triangle.h"
#include <pthread.h>

#include <mat4/type.h>
#include <mat4/create.h>
#include <mat4/identity.h>
#include <mat4/rotateX.h>
#include <mat4/rotateY.h>
#include <mat4/rotateZ.h>
#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>

#include "gl_program.h"
#include "device.h"

#define PATH "./"

#ifndef M_PI
#define M_PI 3.141592654
#endif

typedef struct {
	uint32_t screen_width;
	uint32_t screen_height;
// OpenGL|ES objects
	EGLDisplay display;
	EGLSurface surface;
	EGLContext context;
	struct {
		void *pre_render;
		void *stereo;
	} program;
	GLuint vbo;
	GLuint vbo_nop;
	GLuint tex;
	GLuint pre_render_texture;
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
} CUBE_STATE_T;

static void init_ogl(CUBE_STATE_T *state);
static void init_model_proj(CUBE_STATE_T *state);
static void redraw_pre_render_texture(CUBE_STATE_T *state);
static void redraw_scene(CUBE_STATE_T *state);
static void init_textures(CUBE_STATE_T *state, int image_size_with,
		int image_size_height);
static void exit_func(void);
static volatile int terminate;
static CUBE_STATE_T _state, *state = &_state;

static void* eglImage = 0;
static pthread_t thread1;

/***********************************************************
 * Name: init_ogl
 *
 * Arguments:
 *       CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Sets the display, OpenGL|ES context and screen stuff
 *
 * Returns: void
 *
 ***********************************************************/
static void init_ogl(CUBE_STATE_T *state) {
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

	dispman_display = vc_dispmanx_display_open(0 /* LCD */);
	dispman_update = vc_dispmanx_update_start(0);

	dispman_element = vc_dispmanx_element_add(dispman_update, dispman_display,
			0/*layer*/, &dst_rect, 0/*src*/, &src_rect,
			DISPMANX_PROTECTION_NONE, 0 /*alpha*/, 0/*clamp*/, 0/*transform*/);

	nativewindow.element = dispman_element;
	nativewindow.width = state->screen_width;
	nativewindow.height = state->screen_height;
	vc_dispmanx_update_submit_sync(dispman_update);

	state->surface = eglCreateWindowSurface(state->display, config,
			&nativewindow, NULL);
	assert(state->surface != EGL_NO_SURFACE);

	// connect the context to the surface
	result = eglMakeCurrent(state->display, state->surface, state->surface,
			state->context);
	assert(EGL_FALSE != result);

	//texture rendering
	glGenFramebuffers(1, &state->framebuffer);

	glGenTextures(1, &state->pre_render_texture);
	glBindTexture(GL_TEXTURE_2D, state->pre_render_texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 320, 480, 0, GL_RGB,
			GL_UNSIGNED_BYTE, NULL);
	if (glGetError() != GL_NO_ERROR) {
		printf("glTexImage2D failed. Could not allocate texture buffer.");
	}
	glBindTexture(GL_TEXTURE_2D, 0);

	glBindFramebuffer(GL_FRAMEBUFFER, state->framebuffer);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
			state->pre_render_texture, 0);
	if (glGetError() != GL_NO_ERROR) {
		printf(
				"glFramebufferTexture2D failed. Could not allocate framebuffer.");
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// Set background color and clear buffers
	glClearColor(0.15f, 0.25f, 0.35f, 1.0f);

	// Enable back face culling.
	glEnable(GL_CULL_FACE);
}

int fovmesh(float theta_degree, int phi_degree, int num_of_steps,
		GLuint *vbo_out, GLuint *n_out) {
	GLuint vbo;

	int n = 2 * (num_of_steps - 1) * num_of_steps;
	float points[3 * n];

	float theta = theta_degree * M_PI / 180.0;
	float phi = phi_degree * M_PI / 180.0;

	float x;
	float y;

	float sart_x = -tan(theta / 2);
	float start_y = -tan(phi / 2);

	float end_x = tan(theta / 2);
	float end_y = tan(phi / 2);

	float step_x = (end_x - start_x) / num_of_steps;
	float step_y = (end_y - start_y) / num_of_steps;

	int i = 0;
	for (y = start_y; y < end_y; y += step_y) {
		for (x = sart_x; x <= end_x; x += step_x) {
			{
				float z = 1.0;
				float len = sqrt(x * x + z * z);
				float roll = atan2(y, len);
				float yaw = atan2(x, z);
				points[i++] = cos(roll) * sin(yaw);
				points[i++] = cos(roll) * cos(yaw);
				points[i++] = sin(roll);
			}
			{
				float z = 1.0;
				float len = sqrt(x * x + z * z);
				float roll = atan2(y + step_y, len);
				float yaw = atan2(x, z);
				points[i++] = cos(roll) * sin(yaw);
				points[i++] = cos(roll) * cos(yaw);
				points[i++] = sin(roll);
			}
		}
	}

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * n, points,
			GL_STATIC_DRAW);

	if (vbo_out != NULL)
		*vbo_out = vbo;
	if (n_out != NULL)
		*n_out = n;

	return 0;
}

/***********************************************************
 * Name: init_model_proj
 *
 * Arguments:
 *       CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Sets the OpenGL|ES model to default values
 *
 * Returns: void
 *
 ***********************************************************/
static void init_model_proj(CUBE_STATE_T *state) {
	float fov = 90.0;
	fovmesh(fov, fov * state->screen_height / state->screen_width, 20,
			&state->vbo, &state->vbo_nop);

	state->program.pre_render = GLProgram_new("shader/pre_render.vert",
			"shader/pre_render.frag");
	state->program.stereo = GLProgram_new("shader/stereo.vert",
			"shader/stereo.frag");
}

/***********************************************************
 * Name: redraw_scene
 *
 * Arguments:
 *       CUBE_STATE_T *state - holds OGLES model info
 *
 * Description:   Draws the model and calls eglSwapBuffers
 *                to render to screen
 *
 * Returns: void
 *
 ***********************************************************/
static void redraw_pre_render_texture(CUBE_STATE_T *state) {
	int program = GLProgram_GetId(state->program.pre_render);
	glUseProgram(program);

	glBindFramebuffer(GL_FRAMEBUFFER, state->framebuffer);
	glViewport(0, 0, state->screen_width / 4, state->screen_height / 2);

	glBindBuffer(GL_ARRAY_BUFFER, state->vbo);
	glBindTexture(GL_TEXTURE_2D, state->tex);

	mat4 matrix_camera = mat4_create();
	mat4_identity(matrix_camera);
	mat4_rotateZ(matrix_camera, matrix_camera, 0);
	mat4_rotateY(matrix_camera, matrix_camera, 0);
	mat4_rotateX(matrix_camera, matrix_camera, M_PI / 2);

	mat4 unif_matrix = mat4_create();
	mat4_fromQuat(unif_matrix, get_quatanion());

	mat4_multiply(unif_matrix, matrix_camera, unif_matrix);

	//Load in the texture and thresholding parameters.
	glUniform1i(glGetUniformLocation(program, "tex"), 0);
	glUniformMatrix4fv(glGetUniformLocation(program, "unif_matrix"), 1,
			GL_FALSE, (GLfloat*) unif_matrix);

	GLuint loc = glGetAttribLocation(program, "vPosition");
	glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(loc);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, state->vbo_nop);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
static void redraw_scene(CUBE_STATE_T *state) {
	int program = GLProgram_GetId(state->program.stereo);
	glUseProgram(program);

	// Start with a clear screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBindBuffer(GL_ARRAY_BUFFER, state->vbo);
	glBindTexture(GL_TEXTURE_2D, state->pre_render_texture);

	//Load in the texture and thresholding parameters.
	glUniform1i(glGetUniformLocation(program, "tex"), 0);

	GLuint loc = glGetAttribLocation(program, "vPosition");
	glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(loc);

	//left eye
	//glViewport(0, 0, (GLsizei)state->screen_width/2, (GLsizei)state->screen_height);
	glViewport(state->screen_width / 8, state->screen_height / 4,
			(GLsizei) state->screen_width / 4,
			(GLsizei) state->screen_height / 2);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, state->vbo_nop);

	//right eye
	//glViewport(state->screen_width/2, 0, (GLsizei)state->screen_width/2, (GLsizei)state->screen_height);
	glViewport(state->screen_width / 2 + state->screen_width / 8,
			state->screen_height / 4, (GLsizei) state->screen_width / 4,
			(GLsizei) state->screen_height / 2);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, state->vbo_nop);

	eglSwapBuffers(state->display, state->surface);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
}

/***********************************************************
 * Name: init_textures
 *
 * Arguments:
 *       CUBE_STATE_T *state - holds OGLES model info
 *
 * Description:   Initialise OGL|ES texture surfaces to use image
 *                buffers
 *
 * Returns: void
 *
 ***********************************************************/
static void init_textures(CUBE_STATE_T *state, int image_size_with,
		int image_size_height) {
	//// load three texture buffers but use them on six OGL|ES texture surfaces
	glGenTextures(1, &state->tex);

	glBindTexture(GL_TEXTURE_2D, state->tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_size_with, image_size_height,
			0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	/* Create EGL Image */
	eglImage = eglCreateImageKHR(state->display, state->context,
			EGL_GL_TEXTURE_2D_KHR, (EGLClientBuffer) state->tex, 0);

	if (eglImage == EGL_NO_IMAGE_KHR) {
		printf("eglCreateImageKHR failed.\n");
		exit(1);
	}

	// Start rendering
	pthread_create(&thread1, NULL, video_decode_test, eglImage);

	glEnable(GL_TEXTURE_2D);

	// Bind texture surface to current vertices
	glBindTexture(GL_TEXTURE_2D, state->tex);
}
//------------------------------------------------------------------------------

static void exit_func(void)
// Function to be passed to atexit().
{
	if (eglImage != 0) {
		if (!eglDestroyImageKHR(state->display, (EGLImageKHR) eglImage))
			printf("eglDestroyImageKHR failed.");
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

	printf("\ncube closed\n");
} // exit_func()

//==============================================================================

int main(int argc, char *argv[]) {
	int image_size_with = 1024;
	int image_size_height = 1024;

	bcm_host_init();
	printf("Note: ensure you have sufficient gpu_mem configured\n");

	// Clear application state
	memset(state, 0, sizeof(*state));

	init_device();

	// Start OGLES
	init_ogl(state);

	// Setup the model world
	init_model_proj(state);

	// initialise the OGLES texture(s)
	init_textures(state, image_size_with, image_size_height);

	while (!terminate) {
		redraw_pre_render_texture(state);
		redraw_scene(state);
	}
	exit_func();
	return 0;
}

