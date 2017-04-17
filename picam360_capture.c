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
#include <stdbool.h>
#include <linux/input.h>
#include <fcntl.h>
#include <wchar.h>
#include <pthread.h>

#include "bcm_host.h"

#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"

#include "picam360_capture.h"
#include "video.h"
#include "mjpeg_decoder.h"
#include "video_direct.h"
#include "picam360_tools.h"
#include "gl_program.h"
#include "device.h"
#include "auto_calibration.h"
#include "view_coordinate_mpu9250.h"

//these plugin should be got out to shared object
#include "plugins/driver_agent/driver_agent.h"

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
#include <mat4/invert.h>

//json parser
#include <jansson.h>

#include <opencv/highgui.h>

#define PATH "./"

#ifndef M_PI
#define M_PI 3.141592654
#endif

#define CONFIG_FILE "config.json"

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
static void redraw_info(PICAM360CAPTURE_T *state, FRAME_T *frame);

static volatile int terminate;
static PICAM360CAPTURE_T _state, *state = &_state;
static float lg_fps = 0;

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
		GLuint *vbo_out, GLuint *n_out) {
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
static void init_model_proj(PICAM360CAPTURE_T *state) {
	float maxfov = 150.0;

	board_mesh(&state->model_data[EQUIRECTANGULAR].vbo,
			&state->model_data[EQUIRECTANGULAR].vbo_nop);
	if (state->num_of_cam == 1) {
		state->model_data[EQUIRECTANGULAR].program = GLProgram_new(
				"shader/equirectangular.vert", "shader/equirectangular.frag");
	} else {
		state->model_data[EQUIRECTANGULAR].program = GLProgram_new(
				"shader/equirectangular.vert",
				"shader/equirectangular_sphere.frag");
	}

	board_mesh(&state->model_data[FISHEYE].vbo,
			&state->model_data[FISHEYE].vbo_nop);
	state->model_data[FISHEYE].program = GLProgram_new("shader/fisheye.vert",
			"shader/fisheye.frag");

	board_mesh(&state->model_data[CALIBRATION].vbo,
			&state->model_data[CALIBRATION].vbo_nop);
	state->model_data[CALIBRATION].program = GLProgram_new(
			"shader/calibration.vert", "shader/calibration.frag");

	spherewindow_mesh(maxfov, maxfov, 128, &state->model_data[WINDOW].vbo,
			&state->model_data[WINDOW].vbo_nop);
	if (state->num_of_cam == 1) {
		state->model_data[WINDOW].program = GLProgram_new("shader/window.vert",
				"shader/window.frag");
	} else {
		state->model_data[WINDOW].program = GLProgram_new("shader/window.vert",
				"shader/window_sphere.frag");
	}

	board_mesh(&state->model_data[BOARD].vbo,
			&state->model_data[BOARD].vbo_nop);
	state->model_data[BOARD].program = GLProgram_new("shader/board.vert",
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
		for (int j = 0; j < 2; j++) {
			//// load three texture buffers but use them on six OGL|ES texture surfaces
			glGenTextures(1, &state->cam_texture[i][j]);

			glBindTexture(GL_TEXTURE_2D, state->cam_texture[i][j]);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, state->cam_width,
					state->cam_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

			/* Create EGL Image */
			state->egl_image[i][j] = eglCreateImageKHR(state->display,
					state->context, EGL_GL_TEXTURE_2D_KHR,
					(EGLClientBuffer) state->cam_texture[i][j], 0);

			if (state->egl_image[i][j] == EGL_NO_IMAGE_KHR) {
				printf("eglCreateImageKHR failed.\n");
				exit(1);
			}
		}

		// Start rendering
		if (state->codec_type == MJPEG) {
			init_mjpeg_decoder(&state->plugin_host, i, state->egl_image[i]);
		} else {
			void **args = malloc(sizeof(void*) * 3);
			args[0] = (void*) i;
			args[1] = (void*) state->egl_image[i];
			args[2] = (void*) state;
			pthread_create(&state->thread[i], NULL,
					(state->video_direct) ? video_direct : video_decode_test,
					args);
		}
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
		state->options.sharpness_gain = json_number_value(
				json_object_get(options, "sharpness_gain"));
		for (int i = 0; i < MAX_CAM_NUM; i++) {
			char buff[256];
			sprintf(buff, "cam%d_offset_pitch", i);
			state->options.cam_offset_pitch[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_offset_yaw", i);
			state->options.cam_offset_yaw[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_offset_roll", i);
			state->options.cam_offset_roll[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_offset_x", i);
			state->options.cam_offset_x[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_offset_y", i);
			state->options.cam_offset_y[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_horizon_r", i);
			state->options.cam_horizon_r[i] = json_number_value(
					json_object_get(options, buff));

			if (state->options.cam_horizon_r[i] == 0) {
				state->options.cam_horizon_r[i] = 0.8;
			}
		}
		state->options.view_offset_pitch = json_number_value(
				json_object_get(options, "view_offset_pitch"));
		state->options.view_offset_yaw = json_number_value(
				json_object_get(options, "view_offset_yaw"));
		state->options.view_offset_roll = json_number_value(
				json_object_get(options, "view_offset_roll"));

		if (state->plugins) {
			for (int i = 0; state->plugins[i] != NULL; i++) {
				if (state->plugins[i]->init_options) {
					state->plugins[i]->init_options(
							state->plugins[i]->user_data, options);
				}
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
			json_real(state->options.sharpness_gain));
	for (int i = 0; i < MAX_CAM_NUM; i++) {
		char buff[256];
		sprintf(buff, "cam%d_offset_pitch", i);
		json_object_set_new(options, buff,
				json_real(state->options.cam_offset_pitch[i]));
		sprintf(buff, "cam%d_offset_yaw", i);
		json_object_set_new(options, buff,
				json_real(state->options.cam_offset_yaw[i]));
		sprintf(buff, "cam%d_offset_roll", i);
		json_object_set_new(options, buff,
				json_real(state->options.cam_offset_roll[i]));
		sprintf(buff, "cam%d_offset_x", i);
		json_object_set_new(options, buff,
				json_real(state->options.cam_offset_x[i]));
		sprintf(buff, "cam%d_offset_y", i);
		json_object_set_new(options, buff,
				json_real(state->options.cam_offset_y[i]));
		sprintf(buff, "cam%d_horizon_r", i);
		json_object_set_new(options, buff,
				json_real(state->options.cam_horizon_r[i]));
	}
	json_object_set_new(options, "view_offset_pitch",
			json_real(state->options.view_offset_pitch));
	json_object_set_new(options, "view_offset_yaw",
			json_real(state->options.view_offset_yaw));
	json_object_set_new(options, "view_offset_roll",
			json_real(state->options.view_offset_roll));

	if (state->plugins) {
		for (int i = 0; state->plugins[i] != NULL; i++) {
			if (state->plugins[i]->save_options) {
				state->plugins[i]->save_options(state->plugins[i]->user_data,
						options);
			}
		}
	}

	json_dump_file(options, CONFIG_FILE, 0);

	json_decref(options);
}
//------------------------------------------------------------------------------

static void exit_func(void)
// Function to be passed to atexit().
{
	for (int i = 0; i < state->num_of_cam; i++) {
		for (int j = 0; j < 2; j++) {
			if (state->egl_image[i][j] != 0) {
				if (!eglDestroyImageKHR(state->display,
						(EGLImageKHR) state->egl_image[i][j]))
					printf("eglDestroyImageKHR failed.");
				state->egl_image[i][j] = NULL;
			}
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

static int next_frame_id = 0;

FRAME_T *create_frame(PICAM360CAPTURE_T *state, int argc, char *argv[]) {
	int opt;
	int render_width = 512;
	int render_height = 512;
	FRAME_T *frame = malloc(sizeof(FRAME_T));
	memset(frame, 0, sizeof(FRAME_T));
	frame->id = next_frame_id++;
	frame->operation_mode = WINDOW;
	frame->output_mode = OUTPUT_MODE_NONE;
	frame->view_coordinate_mode = state->default_view_coordinate_mode;
	frame->fov = 120;

	optind = 1; // reset getopt
	while ((opt = getopt(argc, argv, "c:w:h:n:psW:H:ECFDo:i:r:v:")) != -1) {
		switch (opt) {
		case 'W':
			sscanf(optarg, "%d", &render_width);
			break;
		case 'H':
			sscanf(optarg, "%d", &render_height);
			break;
		case 'E':
			frame->operation_mode = EQUIRECTANGULAR;
			break;
		case 'C':
			frame->operation_mode = CALIBRATION;
			break;
		case 'F':
			frame->operation_mode = FISHEYE;
			break;
		case 'o':
			frame->output_mode = OUTPUT_MODE_VIDEO;
			strncpy(frame->output_filepath, optarg,
					sizeof(frame->output_filepath));
			break;
		case 'v':
			if (strcmp(optarg, "MPU9250") == 0) {
				frame->view_coordinate_mode = MPU9250;
			} else if (strcmp(optarg, "OCULUS-RIFT") == 0) {
				frame->view_coordinate_mode = OCULUS_RIFT;
			}
			break;
		default:
			break;
		}
	}

	switch (frame->view_coordinate_mode) {
	case MPU9250:
		init_mpu9250();
		break;
	case OCULUS_RIFT:
		init_device();
		break;
	case MANUAL:
	default:
		break;
	}

	if (render_width > 2048) {
		frame->double_size = true;
		frame->width = render_width / 2;
		frame->height = render_height;
	} else {
		frame->width = render_width;
		frame->height = render_height;
	}

	//texture rendering
	glGenFramebuffers(1, &frame->framebuffer);

	glGenTextures(1, &frame->texture);
	glBindTexture(GL_TEXTURE_2D, frame->texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame->width, frame->height, 0,
			GL_RGB, GL_UNSIGNED_BYTE, NULL);
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

	//buffer memory
	if (frame->double_size) {
		int size = frame->width * frame->height * 3;
		frame->img_buff = (unsigned char*) malloc(size * 2);
	} else {
		int size = frame->width * frame->height * 3;
		frame->img_buff = (unsigned char*) malloc(size);
	}

	return frame;
}

bool delete_frame(FRAME_T *frame) {
	if (frame->befor_deleted_callback) {
		frame->befor_deleted_callback(state, frame);
	}

	if (frame->framebuffer) {
		glDeleteFramebuffers(1, &frame->framebuffer);
		frame->framebuffer = 0;
	}
	if (frame->texture) {
		glDeleteTextures(1, &frame->texture);
		frame->texture = 0;
	}
	if (frame->img_buff) {
		free(frame->img_buff);
		frame->img_buff = NULL;
	}
	free(frame);

	return true;
}

void frame_handler() {
	struct timeval s, f;
	double elapsed_ms;
	bool snap_finished = false;
	FRAME_T **frame_pp = &state->frame;
	while (*frame_pp) {
		FRAME_T *frame = *frame_pp;
		gettimeofday(&s, NULL);

		//start & stop recording
		if (frame->is_recording && frame->output_mode == OUTPUT_MODE_NONE) { //stop record
			StopRecord(frame->recorder);
			frame->recorder = NULL;

			frame->frame_elapsed /= frame->frame_num;
			printf("stop record : frame num : %d : fps %.3lf\n",
					frame->frame_num, 1000.0 / frame->frame_elapsed);

			frame->output_mode = OUTPUT_MODE_NONE;
			frame->is_recording = false;
			frame->delete_after_processed = true;
		}
		if (!frame->is_recording && frame->output_mode == OUTPUT_MODE_VIDEO) {
			int ratio = frame->double_size ? 2 : 1;
			frame->recorder = StartRecord(frame->width * ratio, frame->height,
					frame->output_filepath, 4000 * ratio);
			frame->output_mode = OUTPUT_MODE_VIDEO;
			frame->frame_num = 0;
			frame->frame_elapsed = 0;
			frame->is_recording = true;
			printf("start_record saved to %s\n", frame->output_filepath);
		}

		//rendering to buffer
		int img_width;
		int img_height;
		if (frame->double_size) {
			int size = frame->width * frame->height * 3;
			unsigned char *image_buffer = (unsigned char*) malloc(size);
			unsigned char *image_buffer_double = frame->img_buff;
			img_width = frame->width * 2;
			img_height = frame->height;
			for (int split = 0; split < 2; split++) {
				state->split = split + 1;
				redraw_render_texture(state, frame,
						&state->model_data[frame->operation_mode]);
				glFinish();
				glBindFramebuffer(GL_FRAMEBUFFER, frame->framebuffer);
				glReadPixels(0, 0, frame->width, frame->height, GL_RGB,
						GL_UNSIGNED_BYTE, image_buffer);
				glBindFramebuffer(GL_FRAMEBUFFER, 0);
				for (int y = 0; y < frame->height; y++) {
					memcpy(
							image_buffer_double + frame->width * 2 * 3 * y
									+ frame->width * 3 * split,
							image_buffer + frame->width * 3 * y,
							frame->width * 3);
				}
			}
			free(image_buffer);
		} else {
			unsigned char *image_buffer = frame->img_buff;
			img_width = frame->width;
			img_height = frame->height;
			state->split = 0;
			redraw_render_texture(state, frame,
					&state->model_data[frame->operation_mode]);
			glFinish();
			glBindFramebuffer(GL_FRAMEBUFFER, frame->framebuffer);
			glReadPixels(0, 0, frame->width, frame->height, GL_RGB,
					GL_UNSIGNED_BYTE, image_buffer);
			glBindFramebuffer(GL_FRAMEBUFFER, 0);
		}

		switch (frame->output_mode) {
		case OUTPUT_MODE_STILL:
			SaveJpeg(frame->img_buff, img_width, img_height,
					frame->output_filepath, 70);
			printf("snap saved to %s\n", frame->output_filepath);

			gettimeofday(&f, NULL);
			elapsed_ms = (f.tv_sec - s.tv_sec) * 1000.0
					+ (f.tv_usec - s.tv_usec) / 1000.0;
			printf("elapsed %.3lf ms\n", elapsed_ms);

			frame->output_mode = OUTPUT_MODE_NONE;
			frame->delete_after_processed = true;

			snap_finished = true;
			break;
		case OUTPUT_MODE_VIDEO:
			AddFrame(frame->recorder, frame->img_buff);

			gettimeofday(&f, NULL);
			elapsed_ms = (f.tv_sec - s.tv_sec) * 1000.0
					+ (f.tv_usec - s.tv_usec) / 1000.0;
			frame->frame_num++;
			frame->frame_elapsed += elapsed_ms;
			break;
		default:
			break;
		}
		if (frame->after_processed_callback) {
			frame->after_processed_callback(state, frame);
		}
		//next rendering
		if (frame->delete_after_processed) {
			*frame_pp = frame->next;
			delete_frame(frame);
			frame = NULL;
		} else {
			frame_pp = &frame->next;
		}
		//preview
		if (frame && frame == state->frame && state->preview) {
			redraw_scene(state, frame, &state->model_data[BOARD]);
			if (state->menu_visible) {
				redraw_info(state, frame);
			}
			eglSwapBuffers(state->display, state->surface);
		}
	}
	if (snap_finished) {
		state->plugin_host.send_event(PICAM360_HOST_NODE_ID,
				PICAM360_CAPTURE_EVENT_AFTER_SNAP);
	}
	state->plugin_host.send_event(PICAM360_HOST_NODE_ID,
			PICAM360_CAPTURE_EVENT_AFTER_FRAME);
}

static double calib_step = 0.01;

void _command_handler(const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, "exit", sizeof(buff)) == 0) {
		printf("exit\n");
		exit(0); //temporary
	} else if (strncmp(cmd, "0", sizeof(buff)) == 0) {
		state->active_cam = 0;
	} else if (strncmp(cmd, "1", sizeof(buff)) == 0) {
		state->active_cam = 1;
	} else if (strncmp(cmd, "snap", sizeof(buff)) == 0) {
		char *param = strtok(NULL, "\n");
		if (param != NULL) {
			const int kMaxArgs = 10;
			int argc = 1;
			char *argv[kMaxArgs];
			char *p2 = strtok(param, " ");
			while (p2 && argc < kMaxArgs - 1) {
				argv[argc++] = p2;
				p2 = strtok(0, " ");
			}
			argv[0] = cmd;
			argv[argc] = 0;
			FRAME_T *frame = create_frame(state, argc, argv);
			frame->next = state->frame;
			frame->output_mode = OUTPUT_MODE_STILL;
			frame->view_pitch = 90 * M_PI / 180.0;
			frame->view_yaw = 0;
			frame->view_roll = 0;
			frame->view_coordinate_mode = MANUAL;
			state->frame = frame;
		}
	} else if (strncmp(cmd, "start_record", sizeof(buff)) == 0) {
		char *param = strtok(NULL, "\n");
		if (param != NULL) {
			const int kMaxArgs = 10;
			int argc = 1;
			char *argv[kMaxArgs];
			char *p2 = strtok(param, " ");
			while (p2 && argc < kMaxArgs - 1) {
				argv[argc++] = p2;
				p2 = strtok(0, " ");
			}
			argv[0] = cmd;
			argv[argc] = 0;
			FRAME_T *frame = create_frame(state, argc, argv);
			frame->next = state->frame;
			frame->output_mode = OUTPUT_MODE_VIDEO;
			frame->view_pitch = 90 * M_PI / 180.0;
			frame->view_yaw = 0;
			frame->view_roll = 0;
			frame->view_coordinate_mode = MANUAL;
			state->frame = frame;
			printf("start_record id=%d\n", frame->id);
		}
	} else if (strncmp(cmd, "stop_record", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int id = 0;
			sscanf(param, "%d", &id);
			for (FRAME_T *frame = state->frame; frame != NULL;
					frame = frame->next) {
				if (frame->id == id) {
					frame->output_mode = OUTPUT_MODE_NONE;
					printf("stop_record\n");
					break;
				}
			}
		}
	} else if (strncmp(cmd, "start_ac", sizeof(buff)) == 0) {
		bool checkAcMode = false;
		for (FRAME_T *frame = state->frame; frame != NULL; frame =
				frame->next) {
			if (is_auto_calibration(frame)) {
				checkAcMode = true;
				break;
			}
		}
		if (!checkAcMode) {
			char param[256];
			strncpy(param, "-W 256 -H 256 -F", 256);

			const int kMaxArgs = 10;
			int argc = 1;
			char *argv[kMaxArgs];
			char *p2 = strtok(param, " ");
			while (p2 && argc < kMaxArgs - 1) {
				argv[argc++] = p2;
				p2 = strtok(0, " ");
			}
			argv[0] = "auto_calibration";
			argv[argc] = 0;

			FRAME_T *frame = create_frame(state, argc, argv);
			set_auto_calibration(frame);
			frame->next = state->frame;
			state->frame = frame;

			printf("start_ac\n");
		}
	} else if (strncmp(cmd, "stop_ac", sizeof(buff)) == 0) {
		for (FRAME_T *frame = state->frame; frame != NULL; frame =
				frame->next) {
			if (is_auto_calibration(frame)) {
				frame->delete_after_processed = true;
				printf("stop_ac\n");
				break;
			}
		}
	} else if (strncmp(cmd, "start_record_raw", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL && !state->output_raw) {
			strncpy(state->output_raw_filepath, param,
					sizeof(state->output_raw_filepath) - 1);
			state->output_raw = true;
			printf("start_record_raw saved to %s\n", param);
		}
	} else if (strncmp(cmd, "stop_record_raw", sizeof(buff)) == 0) {
		printf("stop_record_raw\n");
		state->output_raw = false;
	} else if (strncmp(cmd, "load_file", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			strncpy(state->input_filepath, param,
					sizeof(state->input_filepath) - 1);
			state->input_mode = INPUT_MODE_FILE;
			state->input_file_cur = -1;
			state->input_file_size = 0;
			printf("load_file from %s\n", param);
		}
	} else if (strncmp(cmd, "cam_mode", sizeof(buff)) == 0) {
		state->input_mode = INPUT_MODE_CAM;
	} else if (strncmp(cmd, "get_loading_pos", sizeof(buff)) == 0) {
		if (state->input_file_size == 0) {
			printf("%d\n", -1);
		} else {
			double ratio = 100 * state->input_file_cur / state->input_file_size;
			printf("%d\n", (int) ratio);
		}
	} else if (strncmp(cmd, "set_camera_orientation", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float pitch;
			float yaw;
			float roll;
			sscanf(param, "%f,%f,%f", &pitch, &yaw, &roll);
			state->camera_pitch = pitch * M_PI / 180.0;
			state->camera_yaw = yaw * M_PI / 180.0;
			state->camera_roll = roll * M_PI / 180.0;
			printf("set_camera_orientation\n");
		}
	} else if (strncmp(cmd, "set_view_orientation", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int id;
			float pitch;
			float yaw;
			float roll;
			sscanf(param, "%i=%f,%f,%f", &id, &pitch, &yaw, &roll);
			for (FRAME_T *frame = state->frame; frame != NULL;
					frame = frame->next) {
				if (frame->id == id) {
					frame->view_pitch = pitch * M_PI / 180.0;
					frame->view_yaw = yaw * M_PI / 180.0;
					frame->view_roll = roll * M_PI / 180.0;
					frame->view_coordinate_mode = MANUAL;
					printf("set_view_orientation\n");
					break;
				}
			}
		}
	} else if (strncmp(cmd, "set_fov", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int id;
			float fov;
			sscanf(param, "%i=%f", &id, &fov);
			for (FRAME_T *frame = state->frame; frame != NULL;
					frame = frame->next) {
				if (frame->id == id) {
					frame->fov = fov;
					printf("set_fov\n");
					break;
				}
			}
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
	} else if (strncmp(cmd, "set_frame_sync", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			state->frame_sync = (param[0] == '1');
			printf("set_frame_sync %s\n", param);
		}
	} else if (strncmp(cmd, "save", sizeof(buff)) == 0) {
		save_options(state);
	} else if (state->frame != NULL
			&& state->frame->operation_mode == CALIBRATION) {
		if (strncmp(cmd, "step", sizeof(buff)) == 0) {
			char *param = strtok(NULL, " \n");
			if (param != NULL) {
				sscanf(param, "%lf", &calib_step);
			}
		}
		if (strncmp(cmd, "u", sizeof(buff)) == 0
				|| strncmp(cmd, "t", sizeof(buff)) == 0) {
			state->options.cam_offset_y[state->active_cam] += calib_step;
		}
		if (strncmp(cmd, "d", sizeof(buff)) == 0
				|| strncmp(cmd, "b", sizeof(buff)) == 0) {
			state->options.cam_offset_y[state->active_cam] -= calib_step;
		}
		if (strncmp(cmd, "l", sizeof(buff)) == 0) {
			state->options.cam_offset_x[state->active_cam] += calib_step;
		}
		if (strncmp(cmd, "r", sizeof(buff)) == 0) {
			state->options.cam_offset_x[state->active_cam] -= calib_step;
		}
		if (strncmp(cmd, "s", sizeof(buff)) == 0) {
			state->options.sharpness_gain += calib_step;
		}
		if (strncmp(cmd, "w", sizeof(buff)) == 0) {
			state->options.sharpness_gain -= calib_step;
		}
	} else {
		printf("unknown command : %s\n", buff);
	}
}

void command_handler(const char *buff) {
	bool handled = false;
	for (int i = 0; state->plugins[i] != NULL; i++) {
		int name_len = strlen(state->plugins[i]->name);
		if (strncmp(buff, state->plugins[i]->name, name_len) == 0
				&& buff[name_len] == '.') {
			state->plugins[i]->command_handler(state->plugins[i]->user_data,
					buff);
			handled = true;
		}
	}
	if (!handled) {
		_command_handler(buff);
	}
}

void stdin_command_handler() {
	while (inputAvailable()) {
		char buff[256];
		bool error = false;
		for (int i = 0; i < sizeof(buff) - 1; i++) {
			int n = read(STDIN_FILENO, &buff[i], 1);
			if (n == 0) {
				error = true;
				break;
			}
			if (buff[i] == '\n') {
				buff[i + 1] = '\0';
				break;
			}
		}
		if (error) {
			break;
		}
		command_handler(buff);
	}
}

//plugin host methods
static float *get_view_quatanion() {
	switch (state->default_view_coordinate_mode) {
	case MPU9250:
		return get_quatanion_mpu9250();
		break;
	case OCULUS_RIFT:
		return get_quatanion();
		break;
	case MANUAL:
		break;
	}
	return NULL;
}
static void set_view_quatanion(float *value) {
	//TODO
}
static float *get_view_compass() {
	switch (state->default_view_coordinate_mode) {
	case MPU9250:
		return get_compass_mpu9250();
		break;
	case OCULUS_RIFT:
		break;
	case MANUAL:
		break;
	}
	return NULL;
}
static void set_view_compass(float *value) {
	//TODO
}
static float get_view_temperature() {
	switch (state->default_view_coordinate_mode) {
	case MPU9250:
		return get_temperature_mpu9250();
		break;
	case OCULUS_RIFT:
		break;
	case MANUAL:
		break;
	}
	return 0;
}
static void set_view_temperature(float value) {
	//TODO
}
static float get_view_north() {
	switch (state->default_view_coordinate_mode) {
	case MPU9250:
		return get_north_mpu9250();
		break;
	case OCULUS_RIFT:
		break;
	case MANUAL:
		break;
	}
	return 0;
}
static void set_view_north(float value) {
	//TODO
}

static float *get_camera_quatanion(int cam_num) {
	return state->camera_quatanion[cam_num];
}
static void set_camera_quatanion(int cam_num, float *value) {
	for (int i = 0; i < 4; i++) {
		state->camera_quatanion[cam_num][i] = value[i];
	}
	state->camera_coordinate_from_device = true;
}
static float *get_camera_compass() {
	return state->camera_compass;
}
static void set_camera_compass(float *value) {
	for (int i = 0; i < 4; i++) {
		state->camera_compass[i] = value[i];
	}
}
static float get_camera_temperature() {
	return state->camera_temperature;
}
static void set_camera_temperature(float value) {
	state->camera_temperature = value;
}
static float get_camera_north() {
	return state->camera_north;
}
static void set_camera_north(float value) {
	state->camera_north = value;
}
static void decode_video(int cam_num, unsigned char *data, int data_len) {
	if (state->codec_type == MJPEG) {
		mjpeg_decode(cam_num, data, data_len);
	}
}
static void lock_texture() {
	pthread_mutex_lock(&state->texture_mutex);
}
static void unlock_texture() {
	pthread_mutex_unlock(&state->texture_mutex);
}
static void set_cam_texture_cur(int cam_num, int cur) {
	state->cam_texture_cur[cam_num] = cur;
}
static void get_texture_size(uint32_t *width_out, uint32_t *height_out) {
	if (width_out) {
		*width_out = state->cam_width;
	}
	if (height_out) {
		*height_out = state->cam_height;
	}
}
static MENU_T *get_menu() {
	return state->menu;
}
static bool get_menu_visible() {
	return state->menu_visible;
}
static void set_menu_visible(bool value) {
	state->menu_visible = value;
}
static float get_fov() {
	return state->frame->fov;
}
static void set_fov(float value) {
	state->frame->fov = value;
}
static void send_command(const char *cmd) {
	command_handler(cmd);
}
static void send_event(uint32_t node_id, uint32_t event_id) {
	for (int i = 0; state->plugins[i] != NULL; i++) {
		if (state->plugins[i]) {
			state->plugins[i]->event_handler(state->plugins[i]->user_data,
					node_id, event_id);
		}
	}
}
static void snap(int width, int height, enum RENDERING_MODE mode,
		const char *path) {
	char cmd[512];
	char *mode_str = "";
	switch (mode) {
	case RENDERING_MODE_EQUIRECTANGULAR:
		mode_str = "-E";
		break;
	}

	sprintf(cmd, "snap -W %d -H %d %s -o %s", width, height, mode_str, path);
	state->plugin_host.send_command(cmd);
}

static void init_plugins(PICAM360CAPTURE_T *state) {
	{ //init host
		state->plugin_host.get_view_quatanion = get_view_quatanion;
		state->plugin_host.set_view_quatanion = set_view_quatanion;
		state->plugin_host.get_view_compass = get_view_compass;
		state->plugin_host.set_view_compass = set_view_compass;
		state->plugin_host.get_view_temperature = get_view_temperature;
		state->plugin_host.set_view_temperature = set_view_temperature;
		state->plugin_host.get_view_north = get_view_north;
		state->plugin_host.set_view_north = set_view_north;

		state->plugin_host.get_camera_quatanion = get_camera_quatanion;
		state->plugin_host.set_camera_quatanion = set_camera_quatanion;
		state->plugin_host.get_camera_compass = get_camera_compass;
		state->plugin_host.set_camera_compass = set_camera_compass;
		state->plugin_host.get_camera_temperature = get_camera_temperature;
		state->plugin_host.set_camera_temperature = set_camera_temperature;
		state->plugin_host.get_camera_north = get_camera_north;
		state->plugin_host.set_camera_north = set_camera_north;

		state->plugin_host.decode_video = decode_video;
		state->plugin_host.lock_texture = lock_texture;
		state->plugin_host.unlock_texture = unlock_texture;
		state->plugin_host.set_cam_texture_cur = set_cam_texture_cur;
		state->plugin_host.get_texture_size = get_texture_size;

		state->plugin_host.get_menu = get_menu;
		state->plugin_host.get_menu_visible = get_menu_visible;
		state->plugin_host.set_menu_visible = set_menu_visible;

		state->plugin_host.get_fov = get_fov;
		state->plugin_host.set_fov = set_fov;

		state->plugin_host.send_command = send_command;
		state->plugin_host.send_event = send_event;

		state->plugin_host.snap = snap;
	}

	const int INITIAL_SPACE = 16;
	CREATE_PLUGIN create_plugin_funcs[] = { create_driver_agent };
	int num_of_plugins = sizeof(create_plugin_funcs) / sizeof(CREATE_PLUGIN);
	state->plugins = malloc(sizeof(PLUGIN_T*) * INITIAL_SPACE);
	memset(state->plugins, 0, sizeof(PLUGIN_T*) * INITIAL_SPACE);
	for (int i = 0; i < num_of_plugins; i++) {
		create_plugin_funcs[i](&state->plugin_host, &state->plugins[i]);
		state->plugins[i]->node_id = i + 100;
	}
	state->plugins[INITIAL_SPACE - 1] = (void*) -1;
}

void menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {

}

int main(int argc, char *argv[]) {
	pthread_setname_np(pthread_self(), "PICAM360 MAIN");

	bool input_file_mode = false;
	int opt;
	char frame_param[256] = { };

	// Clear application state
	memset(state, 0, sizeof(*state));
	state->cam_width = 1024;
	state->cam_height = 1024;
	state->num_of_cam = 1;
	state->preview = false;
	state->stereo = false;
	state->codec_type = H264;
	state->video_direct = false;
	state->input_mode = INPUT_MODE_CAM;
	state->output_raw = false;

	umask(0000);

	while ((opt = getopt(argc, argv, "c:w:h:n:psd:i:r:F:v:")) != -1) {
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
		case 'd':
			state->video_direct = true;
			break;
		case 'r':
			state->output_raw = true;
			strncpy(state->output_raw_filepath, optarg,
					sizeof(state->output_raw_filepath));
			break;
		case 'i':
			strncpy(state->input_filepath, optarg,
					sizeof(state->input_filepath));
			state->input_mode = INPUT_MODE_FILE;
			state->input_file_cur = -1;
			state->input_file_size = 0;
			state->frame_sync = true;
			input_file_mode = true;
			break;
		case 'F':
			strncpy(frame_param, optarg, 256);
			break;
		case 'v':
			if (strcmp(optarg, "MPU9250") == 0) {
				state->default_view_coordinate_mode = MPU9250;
			} else if (strcmp(optarg, "OCULUS-RIFT") == 0) {
				state->default_view_coordinate_mode = OCULUS_RIFT;
			}
			switch (state->default_view_coordinate_mode) {
			case MPU9250:
				init_mpu9250();
				break;
			case OCULUS_RIFT:
				init_device();
				break;
			case MANUAL:
			default:
				break;
			}
			break;
		default:
			/* '?' */
			printf(
					"Usage: %s [-w width] [-h height] [-n num_of_cam] [-p] [-s]\n",
					argv[0]);
			return -1;
		}
	}

	for (int i = 0; i < MAX_CAM_NUM; i++) {
		mrevent_init(&state->request_frame_event[i]);
		mrevent_trigger(&state->request_frame_event[i]);
		mrevent_init(&state->arrived_frame_event[i]);
		mrevent_reset(&state->arrived_frame_event[i]);
	}

	//texture mutex init
	pthread_mutex_init(&state->texture_mutex, 0);

	bcm_host_init();
	printf("Note: ensure you have sufficient gpu_mem configured\n");

	// Start OGLES
	init_ogl(state);

	//menu
	init_menu(state->screen_height / 32);
	state->menu = menu_new(L"Menu", menu_callback, NULL);

	//frame id=0
	if (frame_param[0]) {
		char *param = frame_param;
		const int kMaxArgs = 10;
		int argc = 1;
		char *argv[kMaxArgs];
		char *p2 = strtok(param, " ");
		while (p2 && argc < kMaxArgs - 1) {
			argv[argc++] = p2;
			p2 = strtok(0, " ");
		}
		argv[0] = "default_frame";
		argv[argc] = 0;
		state->frame = create_frame(state, argc, argv);
	}

	// Setup the model world
	init_model_proj(state);

	// initialise the OGLES texture(s)
	init_textures(state);

	// init plugin
	init_plugins(state);

	//init options
	init_options(state);

	static struct timeval last_time = { };
	gettimeofday(&last_time, NULL);

	while (!terminate) {
		struct timeval time = { };
		gettimeofday(&time, NULL);

		stdin_command_handler();
		if (state->frame_sync) {
			int res = 0;
			for (int i = 0; i < state->num_of_cam; i++) {
				int res = mrevent_wait(&state->arrived_frame_event[i], 1000); //wait 1msec
				if (res != 0) {
					break;
				}
			}
			if (res != 0) {
				continue; // skip
			}
		}
		frame_handler();
		if (state->frame) {
			for (int i = 0; i < state->num_of_cam; i++) {
				mrevent_reset(&state->arrived_frame_event[i]);
				mrevent_trigger(&state->request_frame_event[i]);
			}
		}
		if (input_file_mode
				&& state->input_file_cur == state->input_file_size) {
			terminate = true;
		}

		struct timeval diff;
		timersub(&time, &last_time, &diff);
		float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
		float frame_sec = (((lg_fps != 0) ? 1.0 / lg_fps : 0) * 0.9
				+ diff_sec * 0.1);
		lg_fps = (frame_sec != 0) ? 1.0 / frame_sec : 0;
		last_time = time;
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
	state->plugin_host.lock_texture();

	int program = GLProgram_GetId(model->program);
	glUseProgram(program);

	glBindFramebuffer(GL_FRAMEBUFFER, frame->framebuffer);

	glViewport(0, 0, frame->width, frame->height);

	glBindBuffer(GL_ARRAY_BUFFER, model->vbo);
	glActiveTexture(GL_TEXTURE0);
	if (frame->operation_mode == CALIBRATION) {
		glBindTexture(GL_TEXTURE_2D, state->calibration_texture);
	} else {
		glBindTexture(GL_TEXTURE_2D, state->logo_texture);
	}
	for (int i = 0; i < state->num_of_cam; i++) {
		if (state->codec_type == MJPEG) {
			mjpeg_decoder_switch_buffer(i);
		}
		glActiveTexture(GL_TEXTURE1 + i);
		glBindTexture(GL_TEXTURE_2D,
				state->cam_texture[i][state->cam_texture_cur[i]]);
	}

	//depth axis is z, vertical asis is y
	float unif_matrix[16];
	float unif_matrix_1[16];
	float camera_offset_matrix[16];
	float camera_matrix[16];
	float camera_matrix_1[16];
	float view_offset_matrix[16];
	float view_matrix[16];
	float north_matrix[16];
	float world_matrix[16];
	mat4_identity(unif_matrix);
	mat4_identity(unif_matrix_1);
	mat4_identity(camera_offset_matrix);
	mat4_identity(camera_matrix);
	mat4_identity(camera_matrix_1);
	mat4_identity(view_offset_matrix);
	mat4_identity(view_matrix);
	mat4_identity(north_matrix);
	mat4_identity(world_matrix);

	// Rc : camera orientation

	if (state->camera_coordinate_from_device) {
		mat4_fromQuat(camera_matrix, state->camera_quatanion[0]);
		mat4_fromQuat(camera_matrix_1, state->camera_quatanion[1]);
	} else {
		//euler Y(yaw)X(pitch)Z(roll)
		mat4_rotateZ(camera_matrix, camera_matrix, state->camera_roll);
		mat4_rotateX(camera_matrix, camera_matrix, state->camera_pitch);
		mat4_rotateY(camera_matrix, camera_matrix, state->camera_yaw);

		mat4_rotateZ(camera_matrix, camera_matrix_1, state->camera_roll);
		mat4_rotateX(camera_matrix, camera_matrix_1, state->camera_pitch);
		mat4_rotateY(camera_matrix, camera_matrix_1, state->camera_yaw);
	}

//	// Rco : camera offset
//	//euler Y(yaw)X(pitch)Z(roll)
//	mat4_rotateZ(camera_offset_matrix, camera_offset_matrix,
//			state->options.cam_offset_roll[0]);
//	mat4_rotateX(camera_offset_matrix, camera_offset_matrix,
//			state->options.cam_offset_pitch[0]);
//	mat4_rotateY(camera_offset_matrix, camera_offset_matrix,
//			state->options.cam_offset_yaw[0]);
//
//	mat4_multiply(camera_matrix, camera_matrix, camera_offset_matrix); // Rc'=RcoRc

	// Rvo : view offset
	//euler Y(yaw)X(pitch)Z(roll)
	mat4_rotateZ(view_offset_matrix, view_offset_matrix,
			state->options.view_offset_roll);
	mat4_rotateX(view_offset_matrix, view_offset_matrix,
			state->options.view_offset_pitch);
	mat4_rotateY(view_offset_matrix, view_offset_matrix,
			state->options.view_offset_yaw);

	// Rv : view
	switch (frame->view_coordinate_mode) {
	case MPU9250:
		mat4_fromQuat(view_matrix, get_quatanion_mpu9250());
		mat4_invert(view_matrix, view_matrix);
		break;
	case OCULUS_RIFT:
		mat4_fromQuat(view_matrix, get_quatanion());
		mat4_invert(view_matrix, view_matrix);
		{
			float tmp[16];
			mat4_identity(tmp);
			mat4_rotateX(tmp, tmp, M_PI);
			mat4_multiply(view_matrix, tmp, view_matrix);
		}
		break;
	case MANUAL:
		//euler Y(yaw)X(pitch)Z(roll)
		mat4_rotateZ(view_matrix, view_matrix, frame->view_roll);
		mat4_rotateX(view_matrix, view_matrix, frame->view_pitch);
		mat4_rotateY(view_matrix, view_matrix, frame->view_yaw);
		break;
	}
	mat4_multiply(view_matrix, view_offset_matrix, view_matrix); // Rv=RvRvo

	// Rn
	{
		float north_diff = state->plugin_host.get_view_north()
				- state->plugin_host.get_camera_north();
		mat4_rotateY(north_matrix, north_matrix, north_diff * M_PI / 180);
	}

	// Rw : view coodinate to world coodinate and view heading to ground initially
	mat4_rotateX(world_matrix, world_matrix, -M_PI / 2);

	// Rv : view orientation
	//RcRv(Rc^-1)RcRw
	mat4_multiply(unif_matrix, unif_matrix, world_matrix); // Rw
	mat4_multiply(unif_matrix, unif_matrix, view_matrix); // RvRw
	mat4_multiply(unif_matrix, unif_matrix, north_matrix); // RnRvRw
	mat4_multiply(unif_matrix, unif_matrix, camera_matrix); // RcRnRvRw

	mat4_transpose(unif_matrix, unif_matrix); // this mat4 library is row primary, opengl is column primary

	mat4_multiply(unif_matrix_1, unif_matrix_1, world_matrix); // Rw
	mat4_multiply(unif_matrix_1, unif_matrix_1, view_matrix); // RvRw
	mat4_multiply(unif_matrix_1, unif_matrix_1, north_matrix); // RnRvRw
	mat4_multiply(unif_matrix_1, unif_matrix_1, camera_matrix_1); // RcRnRvRw

	mat4_transpose(unif_matrix_1, unif_matrix_1); // this mat4 library is row primary, opengl is column primary

	//Load in the texture and thresholding parameters.
	glUniform1f(glGetUniformLocation(program, "split"), state->split);
	glUniform1f(glGetUniformLocation(program, "pixel_size"),
			1.0 / state->cam_width);
	{
		float fov_rad = frame->fov * M_PI / 180.0;
		float scale = 1.0 / tan(fov_rad / 2);
		glUniform1f(glGetUniformLocation(program, "scale"), scale);
	}
	{
		float aspect_ratio = (float) state->cam_width
				/ (float) state->cam_height;
		glUniform1f(glGetUniformLocation(program, "aspect_ratio"),
				aspect_ratio);
	}

	glUniform1i(glGetUniformLocation(program, "active_cam"), state->active_cam);

	//options start
	glUniform1f(glGetUniformLocation(program, "sharpness_gain"),
			state->options.sharpness_gain);
	for (int i = 0; i < state->num_of_cam; i++) {
		char buff[256];
		sprintf(buff, "cam%d_offset_yaw", i);
		glUniform1f(glGetUniformLocation(program, buff),
				state->options.cam_offset_yaw[i]);
		sprintf(buff, "cam%d_offset_x", i);
		glUniform1f(glGetUniformLocation(program, buff),
				state->options.cam_offset_x[i]);
		sprintf(buff, "cam%d_offset_y", i);
		glUniform1f(glGetUniformLocation(program, buff),
				state->options.cam_offset_y[i]);
		sprintf(buff, "cam%d_horizon_r", i);
		glUniform1f(glGetUniformLocation(program, buff),
				state->options.cam_horizon_r[i]);
	}
	glUniform1f(glGetUniformLocation(program, "cam_offset_yaw"),
			state->options.cam_offset_yaw[state->active_cam]);
	glUniform1f(glGetUniformLocation(program, "cam_offset_x"),
			state->options.cam_offset_x[state->active_cam]);
	glUniform1f(glGetUniformLocation(program, "cam_offset_y"),
			state->options.cam_offset_y[state->active_cam]);
	glUniform1f(glGetUniformLocation(program, "cam_horizon_r"),
			state->options.cam_horizon_r[state->active_cam]);
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
	glUniformMatrix4fv(glGetUniformLocation(program, "unif_matrix_1"), 1,
			GL_FALSE, (GLfloat*) unif_matrix_1);

	GLuint loc = glGetAttribLocation(program, "vPosition");
	glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(loc);

	glDisable(GL_BLEND);
	glEnable(GL_CULL_FACE);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, model->vbo_nop);
	glFlush();

	glDisableVertexAttribArray(loc);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	state->plugin_host.unlock_texture();
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

	glDisable(GL_BLEND);
	glEnable(GL_CULL_FACE);

	if (frame->operation_mode == CALIBRATION) {
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

	glDisableVertexAttribArray(loc);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
}

static void redraw_info(PICAM360CAPTURE_T *state, FRAME_T *frame) {
	int len = 0;
	const int MAX_INFO_SIZE = 1024;
	wchar_t disp[MAX_INFO_SIZE];
	len += swprintf(disp, MAX_INFO_SIZE, L"Temp %.1f degC",
			state->plugin_host.get_camera_temperature());
	len += swprintf(disp + len, MAX_INFO_SIZE - len, L"\nfps %.1f", lg_fps);
	for (int i = 0; state->plugins[i] != NULL; i++) {
		if (state->plugins[i]->get_info) {
			wchar_t *info = state->plugins[i]->get_info(
					state->plugins[i]->user_data);
			len += swprintf(disp + len, MAX_INFO_SIZE - len, L"\n%ls", info);
		}
	}
	menu_redraw(state->menu, disp, state->screen_width, state->screen_height,
			frame->width, frame->height, state->stereo);
}

