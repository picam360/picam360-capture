#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <pthread.h>
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <limits.h>

#include "egl_handler.h"

#define CONTEXT_SHARING

#ifdef BCM_HOST
#include "bcm_host.h"
#endif

#ifdef USE_GLEW
#include <GL/glew.h>
#endif

#ifdef USE_GLFW
#include <GLFW/glfw3.h>
#endif

static void glfwErrorCallback(int num, const char *err_str) {
	printf("GLFW Error: %s\n", err_str);
}

void init_egl(EGL_HANDLER_T *state) {
#ifdef USE_GLES
#ifdef USE_GLFW
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	glfwSetErrorCallback(glfwErrorCallback);
	if (glfwInit() == GL_FALSE) {
		printf("error on glfwInit\n");
	}
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	state->screen_width = 640;
	state->screen_height = 480;
	state->glfw_window = glfwCreateWindow(state->screen_width, state->screen_height, "picam360", NULL, NULL);

	glfwMakeContextCurrent(state->glfw_window);

#ifdef USE_GLEW
	glewExperimental = GL_TRUE; //avoid glGenVertexArrays crash with glew-1.13
	if (glewInit() != GLEW_OK) {
		printf("error on glewInit\n");
	}
#endif
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
#elif BCM_HOST

	bcm_host_init();
	printf("Note: ensure you have sufficient gpu_mem configured\n");

	int32_t success = 0;
	EGLBoolean result;
	EGLint num_config;

	static EGL_DISPMANX_WINDOW_T nativewindow;

	DISPMANX_ELEMENT_HANDLE_T dispman_element;
	DISPMANX_DISPLAY_HANDLE_T dispman_display;
	DISPMANX_UPDATE_HANDLE_T dispman_update;
	VC_RECT_T dst_rect;
	VC_RECT_T src_rect;

	static const EGLint attribute_list[] = {EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_DEPTH_SIZE, 16,
		//EGL_SAMPLES, 4,
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT, EGL_NONE};
	static const EGLint context_attributes[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};

	EGLConfig config;

	// get an EGL display connection
	state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	assert(state->display != EGL_NO_DISPLAY);

	// initialize the EGL display connection
	result = eglInitialize(state->display, NULL, NULL);
	assert(EGL_FALSE != result);

	// get an appropriate EGL frame buffer configuration
	// this uses a BRCM extension that gets the closest match, rather than standard which returns anything that matches
	result = eglSaneChooseConfigBRCM(state->display, attribute_list, &config, 1, &num_config);
	assert(EGL_FALSE != result);

	//Bind to the right EGL API.
	result = eglBindAPI(EGL_OPENGL_ES_API);
	assert(result != EGL_FALSE);

	// create an EGL rendering context
	state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT, context_attributes);
	assert(state->context != EGL_NO_CONTEXT);

	// create an EGL window surface
	success = graphics_get_display_size(0 /* LCD */, &state->screen_width, &state->screen_height);
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

		dispman_element = vc_dispmanx_element_add(dispman_update, dispman_display, 0/*layer*/, &dst_rect, 0/*src*/, &src_rect, DISPMANX_PROTECTION_NONE, 0 /*alpha*/, 0/*clamp*/, 0/*transform*/);

		nativewindow.element = dispman_element;
		nativewindow.width = state->screen_width;
		nativewindow.height = state->screen_height;
		vc_dispmanx_update_submit_sync(dispman_update);

		state->surface = eglCreateWindowSurface(state->display, config, &nativewindow, NULL);
	}
//	else {
//		//Create an offscreen rendering surface
//		EGLint rendering_attributes[] = { EGL_WIDTH, state->screen_width,
//				EGL_HEIGHT, state->screen_height, EGL_NONE };
//		state->surface = eglCreatePbufferSurface(state->display, config,
//				rendering_attributes);
//	}
	assert(state->surface != EGL_NO_SURFACE);
#elif TEGRA
	EGLBoolean result;
	EGLint num_config;
	static const EGLint attribute_list[] = {
		EGL_RED_SIZE, 8, //
		EGL_GREEN_SIZE, 8,//
		EGL_BLUE_SIZE, 8,//
		EGL_ALPHA_SIZE, 8,//
		EGL_DEPTH_SIZE, 16,//
		//EGL_SAMPLES, 4,
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,//
		EGL_NONE};
	static const EGLint context_attributes[] = {
		EGL_CONTEXT_CLIENT_VERSION, 2, //
		EGL_NONE};

	// get an EGL display connection
	state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	assert(state->display != EGL_NO_DISPLAY);

	// initialize the EGL display connection
	result = eglInitialize(state->display, NULL, NULL);
	assert(EGL_FALSE != result);

	// get an appropriate EGL frame buffer configuration
	result = eglChooseConfig(state->display, attribute_list, &state->config, 1, &num_config);
	assert(EGL_FALSE != result);

	//Bind to the right EGL API.
	result = eglBindAPI(EGL_OPENGL_ES_API);
	assert(result != EGL_FALSE);

	// create an EGL rendering context
	state->context = eglCreateContext(state->display, state->config, EGL_NO_CONTEXT, context_attributes);
	assert(state->context != EGL_NO_CONTEXT);

	//Create an offscreen rendering surface
	EGLint rendering_attributes[] = {EGL_WIDTH, state->screen_width,
		EGL_HEIGHT, state->screen_height, EGL_NONE};
	state->surface = eglCreatePbufferSurface(state->display, state->config,
			rendering_attributes);

	state->screen_width = 640;
	state->screen_height = 480;
#endif

#else
	state->screen_width = 640;
	state->screen_height = 480;
#endif

	eh_activate_context(state);
}

void eh_activate_context(EGL_HANDLER_T *handler) {
#ifdef USE_GLFW
	return;
#elif USE_GLES
#ifdef CONTEXT_SHARING
	EGLBoolean result;
	result = eglMakeCurrent(handler->display, handler->surface, handler->surface, handler->context);
	assert(EGL_FALSE != result);
#endif
#endif
}
void eh_deactivate_context(EGL_HANDLER_T *handler) {
#ifdef USE_GLFW
	return;
#elif USE_GLES
#ifdef CONTEXT_SHARING
	EGLBoolean result;
	result = eglMakeCurrent(handler->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	assert(EGL_FALSE != result);
#endif
#endif
}
void* eh_get_context(EGL_HANDLER_T *handler) {
#ifdef USE_GLFW
	return NULL;
#else
	return handler->context;
#endif
}
void* eh_get_display(EGL_HANDLER_T *handler) {
#ifdef USE_GLFW
	return NULL;
#else
	return handler->display;
#endif
}
