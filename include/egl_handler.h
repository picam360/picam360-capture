#pragma once

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef USE_GLES
#include "EGL/egl.h"
#include "EGL/eglext.h"
#endif

typedef struct _EGL_HANDLER_T {
	int32_t screen_width;
	int32_t screen_height;
// OpenGL|ES objects
#ifdef USE_GLES
	pthread_t context_tid;
	EGLContext context_tmp;
	EGLDisplay display;
	EGLConfig config;
	EGLSurface surface;
	EGLContext context;
#else
	GLFWwindow *glfw_window;
#endif
} EGL_HANDLER_T;

void init_egl(EGL_HANDLER_T *handler);
void eh_activate_context(EGL_HANDLER_T *handler);
void eh_deactivate_context(EGL_HANDLER_T *handler);
