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
#include <fcntl.h>
#include <pthread.h>
#include <editline/readline.h>
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <uuid/uuid.h>
#include <limits.h>
#include <wordexp.h>

#include "tools.h"
#include "picam360_capture.h"
#include "stream_mixer.h"
#include "manual_mpu.h"
#include "png_loader.h"
#include "img/logo_png.h"

//handlers
#include "status_handler.h"
#include "menu_handler.h"

#include <mat4/type.h>
//#include <mat4/create.h>
#include <mat4/identity.h>
#include <mat4/rotateX.h>
#include <mat4/rotateY.h>
#include <mat4/rotateZ.h>
//#include <mat4/scale.h>
#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
//#include <mat4/perspective.h>
#include <mat4/invert.h>
#include <mat4/determinant.h>

//json parser
#include <jansson.h>

#ifdef _WIN64
//define something for Windows (64-bit)
#elif _WIN32
//define something for Windows (32-bit)
#elif __APPLE__
#include "TargetConditionals.h"
#if TARGET_OS_IPHONE && TARGET_IPHONE_SIMULATOR
// define something for simulator
#elif TARGET_OS_IPHONE
// define something for iphone
#else
#define TARGET_OS_OSX 1
// define something for OSX
#endif
#elif __linux
// linux
#include <editline/history.h>
#elif __unix // all unices not caught above
// Unix
#elif __posix
// POSIX
#endif

#define PATH "./"
#define PICAM360_HISTORY_FILE ".picam360_history"
#define PLUGIN_NAME "picam360"

#ifndef M_PI
#define M_PI 3.141592654
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static void init_options(PICAM360CAPTURE_T *state);
static void save_options(PICAM360CAPTURE_T *state);
static void init_options_ex(PICAM360CAPTURE_T *state);
static void save_options_ex(PICAM360CAPTURE_T *state);
static void exit_func(void);

static void loading_callback(void *user_data, int ret);

static volatile int terminate;
static PICAM360CAPTURE_T _state, *state = &_state;

static float lg_cam_fps[MAX_CAM_NUM] = { };
static float lg_cam_frameskip[MAX_CAM_NUM] = { };
static float lg_cam_bandwidth = 0;

static void loading_callback(void *user_data, int ret) {
	printf("end of loading\n");
}

static json_t* json_load_file_without_comment(const char *path, size_t flags,
		json_error_t *error) {
	int ret;
	json_t *options;
	char buff[1024];
	char *tmp_conf_filepath = "/tmp/picam360-capture.conf.json";
	snprintf(buff, sizeof(buff), "grep -v -e '^\\s*#' %s > %s", path,
			tmp_conf_filepath);
	ret = system(buff);
	options = json_load_file(tmp_conf_filepath, flags, error);
	return options;
}

static int end_width(const char *str, const char *suffix) {
	if (!str || !suffix)
		return 0;
	size_t lenstr = strlen(str);
	size_t lensuffix = strlen(suffix);
	if (lensuffix > lenstr)
		return 0;
	return strncmp(str + lenstr - lensuffix, suffix, lensuffix) == 0;
}

static void glfwErrorCallback(int num, const char *err_str) {
	printf("GLFW Error: %s\n", err_str);
}

static int load_texture(const char *filename, uint32_t *tex_out) {
	GLenum err;
	GLuint tex;
	PICAM360_IMAGE_T image = { };

	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width[0], image.height[0], 0,
	GL_RGB, GL_UNSIGNED_BYTE, image.pixels[0]);
	if ((err = glGetError()) != GL_NO_ERROR) {
#ifdef USE_GLES
		printf("glTexImage2D failed. Could not allocate texture buffer.\n");
#else
		printf("glTexImage2D failed. Could not allocate texture buffer. %s\n",
				gluErrorString(err));
#endif
	}
	glBindTexture(GL_TEXTURE_2D, 0);
	if (tex_out != NULL)
		*tex_out = tex;

	return 0;
}

static void get_logo_image(PICAM360_IMAGE_T *img) {
	if (state->logo_image.pixels[0] == NULL) {
		//init logo
		int ret;
		const char *tmp_filepath = "/tmp/tmp.png";
		int fd = open(tmp_filepath, O_CREAT | O_WRONLY | O_TRUNC,
				S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		ret = write(fd, logo_png, logo_png_len);
		close(fd);

		memcpy(state->logo_image.img_type, "RGBA", 4);
		state->logo_image.num_of_planes = 1;
		load_png(tmp_filepath, &state->logo_image.pixels[0],
				&state->logo_image.width[0], &state->logo_image.height[0],
				&state->logo_image.stride[0]);

		remove(tmp_filepath);
	}
	if (img) {
		*img = state->logo_image;
	}
}

static void get_rendering_params(VECTOR4D_T view_quat,
		RENDERING_PARAMS_T *params) {
	{
		//params->stereo = frame->stereo;
		//params->fov = frame->fov;
		params->active_cam = state->active_cam;
		params->num_of_cam = state->num_of_cam;
	}
	{ //cam_attitude //depth axis is z, vertical asis is y
		float view_matrix[16];
		float north_matrix[16];
		float world_matrix[16];

		{ // Rv : view
			mat4_identity(view_matrix);
			mat4_fromQuat(view_matrix, view_quat.ary);
			mat4_invert(view_matrix, view_matrix);
		}

		{ // Rn : north
			mat4_identity(north_matrix);
			float north = state->plugin_host.get_view_north();
			mat4_rotateY(north_matrix, north_matrix, north * M_PI / 180);
		}

		{ // Rw : view coodinate to world coodinate and view heading to ground initially
			mat4_identity(world_matrix);
			mat4_rotateX(world_matrix, world_matrix, -M_PI / 2);
		}

		for (int i = 0; i < state->num_of_cam; i++) {
			float *unif_matrix = params->cam_attitude[i];
			float cam_matrix[16];
			{ // Rc : cam orientation
				mat4_identity(cam_matrix);
				if (state->camera_coordinate_from_device) {
					mat4_fromQuat(cam_matrix, state->camera_quaternion[i].ary);
				} else {
					//euler Y(yaw)X(pitch)Z(roll)
					mat4_rotateZ(cam_matrix, cam_matrix, state->camera_roll);
					mat4_rotateX(cam_matrix, cam_matrix, state->camera_pitch);
					mat4_rotateY(cam_matrix, cam_matrix, state->camera_yaw);
				}
			}

			{ // Rco : cam offset  //euler Y(yaw)X(pitch)Z(roll)
				float *cam_offset_matrix = params->cam_offset_matrix[i];
				mat4_identity(cam_offset_matrix);
				mat4_rotateZ(cam_offset_matrix, cam_offset_matrix,
						state->options.cam_offset_roll[i]);
				mat4_rotateX(cam_offset_matrix, cam_offset_matrix,
						state->options.cam_offset_pitch[i]);
				mat4_rotateY(cam_offset_matrix, cam_offset_matrix,
						state->options.cam_offset_yaw[i]);
				mat4_invert(cam_offset_matrix, cam_offset_matrix);
				mat4_multiply(cam_matrix, cam_matrix, cam_offset_matrix); // Rc'=RcoRc
			}

			{ //RcRv(Rc^-1)RcRw
				mat4_identity(unif_matrix);
				mat4_multiply(unif_matrix, unif_matrix, world_matrix); // Rw
				mat4_multiply(unif_matrix, unif_matrix, view_matrix); // RvRw
				//mat4_multiply(unif_matrix, unif_matrix, north_matrix); // RnRvRw
				mat4_multiply(unif_matrix, unif_matrix, cam_matrix); // RcRnRvRw
			}
			mat4_transpose(unif_matrix, unif_matrix); // this mat4 library is row primary, opengl is column primary
//			{
//				//normalize det = 1
//				float det = mat4_determinant(unif_matrix);
//				float det_4 = pow(det, 0.25);
//				for (int i = 0; i < 16; i++) {
//					unif_matrix[i] / det_4;
//				}
//				//printf("det=%f, %f, %f\n", det, pow(det, 0.25), mat4_determinant(unif_matrix));
//			}
		}
	}
	{ //cam_options
		for (int i = 0; i < state->num_of_cam; i++) {
			params->cam_offset_x[i] = state->options.cam_offset_x[i];
			params->cam_offset_y[i] = state->options.cam_offset_y[i];
			params->cam_horizon_r[i] = state->options.cam_horizon_r[i];
			params->cam_aov[i] = state->options.cam_aov[i];
			if (state->options.config_ex_enabled) {
				params->cam_offset_x[i] += state->options.cam_offset_x_ex[i];
				params->cam_offset_y[i] += state->options.cam_offset_y_ex[i];
				params->cam_horizon_r[i] += state->options.cam_horizon_r_ex[i];
			}
			params->cam_horizon_r[i] *= state->camera_horizon_r_bias;
			params->cam_aov[i] /= state->refraction;
		}
	}
}

static bool _destroy_vstream(VSTREAMER_T *_stream) {
	for (VSTREAMER_T *stream = _stream; stream != NULL;) {
		VSTREAMER_T *tmp = stream;
		stream = stream->next_streamer;
		tmp->release(tmp);
	}
	return true;
}

static int mixer_event_callback(void *user_data, enum STREAM_MIXER_EVENT event) {
	STREAM_MIXER_T *mixer = (STREAM_MIXER_T*) user_data;
	if (event == STREAM_MIXER_EVENT_ALL_OUTPUT_RELEASED) {
		VSTREAMER_T *streamer = NULL;
		while (mixer->get_input(mixer, 0, &streamer) > 0) {
			for (; streamer->pre_streamer != NULL;
					streamer = streamer->pre_streamer) {
				//do nothing
			}
			_destroy_vstream(streamer);
		}
		LIST_T **pp;
		LIST_FOR_START(pp, STREAM_MIXER_T, _mixer, state->stream_mixer_list)
			if (_mixer == mixer) {
				mixer->release(mixer);
				LIST_DELETE(pp);
				break;
			}
		}
	}
}

static VSTREAMER_T* _build_vstream(PICAM360CAPTURE_T *state, const char *_buff);
static STREAM_MIXER_T* _build_mixer(PICAM360CAPTURE_T *state, const char *name) {
	STREAM_MIXER_DEF_T *mixer_def = NULL;
	LIST_T **pp;
	LIST_FOR_START(pp, STREAM_MIXER_DEF_T, _mixer_def, state->stream_mixer_def_list)
		if (strcmp(_mixer_def->name, name) == 0) {
			mixer_def = _mixer_def;
			break;
		}
	}
	if (mixer_def == NULL) {
		return NULL;
	}

	STREAM_MIXER_T *mixer = NULL;
	create_stream_mixer(&mixer);
	strcpy(mixer->name, name);
	mixer->event_callback = mixer_event_callback;

	for (int i = 0; mixer_def->vistreams[i]; i++) {
		char buff[256];
		strncpy(buff, mixer_def->vistreams[i], sizeof(buff));

		VSTREAMER_T *vistream = _build_vstream(state, buff);
		if (vistream == NULL) {
			printf("build stream failed : %s\n", buff);
			continue;
		}
		//connect mixer input
		VSTREAMER_T *pre = NULL;
		VSTREAMER_T **tail_p = &vistream;
		for (; (*tail_p) != NULL; tail_p = &(*tail_p)->next_streamer) {
			if ((*tail_p)->next_streamer == NULL) {
				pre = (*tail_p);
			}
		}
		mixer->create_input(mixer, tail_p);
		(*tail_p)->pre_streamer = pre;

		vistream->start(vistream);
	}

	return mixer;
}

static VSTREAMER_T* _build_vstream(PICAM360CAPTURE_T *state, const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));

	const int kMaxArgs = 32;
	int argc = 0;
	char *argv[kMaxArgs];
	char *p = strtok(buff, "!");
	while (p && argc < kMaxArgs - 1) {
		argv[argc++] = p;
		p = strtok(0, "!");
	}
	VSTREAMER_T *streamer = NULL;
	VSTREAMER_T *pre_streamer = NULL;
	VSTREAMER_T **streamer_p = &streamer;
	for (int s = 0; s < argc; s++) {
		int argc2 = 0;
		char *argv2[kMaxArgs];
		char *p2 = strtok(argv[s], " ");
		while (p2 && argc2 < kMaxArgs - 1) {
			argv2[argc2++] = p2;
			p2 = strtok(0, " ");
		}
		if (strcmp(argv2[0], "mixer") == 0) {
			char mixer_name[64] = { };
			for (int p = 1; p < argc2; p++) {
				char name[64] = { };
				char value[512] = { };
				int len = strlen(argv2[p]);
				for (int i = 0; argv2[p][i] != '\0'; i++) {
					if (argv2[p][i] == '=' && i < sizeof(name)
							&& len - (i + 1) < sizeof(value)) {
						memcpy(name, argv2[p], i);
						memcpy(value, argv2[p] + (i + 1), len - (i + 1));
						if (strcmp(name, "name") == 0) {
							p = argc2;
							strcpy(mixer_name, value);
						}
						break;
					}
				}
			}
			if (mixer_name[0] == '\0') {
				strcpy(mixer_name, "default");
			}
			STREAM_MIXER_T *mixer = NULL;
			LIST_T **pp;
			LIST_FOR_START(pp, STREAM_MIXER_T, _mixer, state->stream_mixer_list)
				if (strcmp(_mixer->name, mixer_name) == 0) {
					mixer = _mixer;
					break;
				}
			}
			if (mixer == NULL) {
				mixer = _build_mixer(state, mixer_name);
				if (mixer == NULL) {
					printf("build mixer failed : %s\n", mixer_name);
					return NULL;
				}
				LIST_T **pp;
				LIST_TAIL(pp, state->stream_mixer_list);
				LIST_NEW(pp, mixer);
			}
			mixer->create_output(mixer, streamer_p);
			(*streamer_p)->pre_streamer = pre_streamer;
			pre_streamer = (*streamer_p);
			streamer_p = &(*streamer_p)->next_streamer;
		} else {
			bool supported = false;
			for (int j = 0; state->vstreamer_factories[j] != NULL; j++) {
				if (strncmp(state->vstreamer_factories[j]->name, argv2[0],
						sizeof(state->vstreamer_factories[j]->name)) == 0) {
					state->vstreamer_factories[j]->create_vstreamer(
							state->vstreamer_factories[j], streamer_p);
					if ((*streamer_p) == NULL) {
						break;
					}
					for (int p = 1; p < argc2; p++) {
						char *name = strtok(argv2[p], "=");
						char *value = strtok(NULL, "=");
						(*streamer_p)->set_param((*streamer_p), name, value);
					}
					(*streamer_p)->pre_streamer = pre_streamer;
					pre_streamer = (*streamer_p);
					streamer_p = &(*streamer_p)->next_streamer;

					supported = true;
					break;
				}
			}
			if (supported == false) {
				printf("%s : not supported\n", argv2[0]);
			}
		}
	}

	return streamer;
}

static VSTREAMER_T* build_vstream(uuid_t uuid, const char *buff) {
	VSTREAMER_T *vstreamer = _build_vstream(state, buff);
	memcpy(vstreamer->uuid, uuid, sizeof(uuid_t));

	LIST_T **pp;
	LIST_TAIL(pp, state->vostream_list);
	LIST_NEW(pp, vstreamer);

	return vstreamer;
}

static bool destroy_vstream(uuid_t uuid) {
	LIST_T **pp;
	LIST_FOR_START(pp, VSTREAMER_T, streamer, state->vostream_list)
		if (uuid_compare(streamer->uuid, uuid) == 0) {
			_destroy_vstream(streamer);
			LIST_DELETE(pp);
			return true;
		}
	}
	return false;
}

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
	json_t *options = json_load_file_without_comment(state->config_filepath, 0,
			&error);
	if (options == NULL) {
		fputs(error.text, stderr);
		exit(1);
	}
	{ //stream_mixer_defs
		json_t *obj = json_object_get(options, "stream_mixer_defs");
		if (obj && json_is_array(obj)) {
			for (int i = 0; i < json_array_size(obj); i++) {
				STREAM_MIXER_DEF_T *mixer_def = (STREAM_MIXER_DEF_T*) malloc(
						sizeof(STREAM_MIXER_DEF_T));
				memset(mixer_def, 0, sizeof(STREAM_MIXER_DEF_T));
				json_t *i_obj = json_array_get(obj, i);

				if (i == 0) {
					mixer_def->name = "default";
				} else {
					json_t *name_obj = json_object_get(i_obj, "name");
					if (name_obj) {
						int len = json_string_length(name_obj);
						mixer_def->name = (char*) malloc(
								sizeof(char) * (len + 1));
						memset(mixer_def->name, 0, sizeof(char) * (len + 1));
						strncpy(mixer_def->name, json_string_value(name_obj),
								len);
					}
				}
				json_t *vistreams_obj = json_object_get(i_obj, "vistreams");
				if (vistreams_obj && json_is_array(vistreams_obj)) {
					mixer_def->vistreams = (char**) malloc(
							sizeof(char**)
									* (json_array_size(vistreams_obj) + 1));
					memset(mixer_def->vistreams, 0,
							sizeof(char**)
									* (json_array_size(vistreams_obj) + 1));
					for (int j = 0; j < json_array_size(vistreams_obj); j++) {
						json_t *j_obj = json_array_get(vistreams_obj, j);
						int len = json_string_length(j_obj);
						mixer_def->vistreams[j] = (char*) malloc(
								sizeof(char) * (len + 1));
						memset(mixer_def->vistreams[j], 0,
								sizeof(char) * (len + 1));
						strncpy(mixer_def->vistreams[j],
								json_string_value(j_obj), len);
					}
				}
				LIST_T **pp;
				LIST_TAIL(pp, state->stream_mixer_def_list);
				LIST_NEW(pp, mixer_def);
			}
		}
	}
	state->num_of_cam = json_number_value(
			json_object_get(options, "num_of_cam"));
	state->num_of_cam = MAX(0, MIN(state->num_of_cam, MAX_CAM_NUM));
	{
		json_t *value = json_object_get(options, "mpu_name");
		if (value) {
			strncpy(state->mpu_name, json_string_value(value),
					sizeof(state->mpu_name) - 1);
		}
	}
	state->options.sharpness_gain = json_number_value(
			json_object_get(options, "sharpness_gain"));
	state->options.color_offset = json_number_value(
			json_object_get(options, "color_offset"));
	state->options.overlap = json_number_value(
			json_object_get(options, "overlap"));
	for (int i = 0; i < state->num_of_cam; i++) {
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
		sprintf(buff, "cam%d_aov", i);
		state->options.cam_aov[i] = json_number_value(
				json_object_get(options, buff));

		if (state->options.cam_horizon_r[i] == 0) {
			state->options.cam_horizon_r[i] = 0.8;
		}
		if (state->options.cam_aov[i] == 0) {
			state->options.cam_aov[i] = 245;
		}
	}
	{ //rtp
		state->options.rtp_rx_port = json_number_value(
				json_object_get(options, "rtp_rx_port"));
		state->options.rtp_rx_type = rtp_get_rtp_socket_type(
				json_string_value(json_object_get(options, "rtp_rx_type")));
		json_t *value = json_object_get(options, "rtp_tx_ip");
		if (value) {
			strncpy(state->options.rtp_tx_ip, json_string_value(value),
					sizeof(state->options.rtp_tx_ip) - 1);
		} else {
			memset(state->options.rtp_tx_ip, 0,
					sizeof(state->options.rtp_tx_ip));
		}
		state->options.rtp_tx_port = json_number_value(
				json_object_get(options, "rtp_tx_port"));
		state->options.rtp_tx_type = rtp_get_rtp_socket_type(
				json_string_value(json_object_get(options, "rtp_tx_type")));
	}
	{ //rtcp
		state->options.rtcp_rx_port = json_number_value(
				json_object_get(options, "rtcp_rx_port"));
		state->options.rtcp_rx_type = rtp_get_rtp_socket_type(
				json_string_value(json_object_get(options, "rtcp_rx_type")));
		json_t *value = json_object_get(options, "rtcp_tx_ip");
		if (value) {
			strncpy(state->options.rtcp_tx_ip, json_string_value(value),
					sizeof(state->options.rtcp_tx_ip) - 1);
		} else {
			memset(state->options.rtcp_tx_ip, 0,
					sizeof(state->options.rtcp_tx_ip));
		}
		state->options.rtcp_tx_port = json_number_value(
				json_object_get(options, "rtcp_tx_port"));
		state->options.rtcp_tx_type = rtp_get_rtp_socket_type(
				json_string_value(json_object_get(options, "rtcp_tx_type")));
	}
	state->options.is_samplerExternalOES = json_number_value(
			json_object_get(options, "is_samplerExternalOES"));

	json_decref(options);
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

	{ //stream_mixer_defs
		json_t *obj = json_array();
		LIST_T **pp;
		LIST_FOR_START(pp, STREAM_MIXER_DEF_T, mixer_def, state->stream_mixer_def_list)
			json_t *i_obj = json_object();
			json_object_set_new(i_obj, "name", json_string(mixer_def->name));
			json_t *j_obj = json_array();
			for (int j = 0; mixer_def->vistreams[j]; j++) {
				json_array_append_new(j_obj,
						json_string(mixer_def->vistreams[j]));
			}
			json_object_set_new(i_obj, "vistreams", j_obj);
			json_array_append_new(obj, i_obj);
		}
		json_object_set_new(options, "stream_mixer_defs", obj);
	}
	json_object_set_new(options, "num_of_cam", json_integer(state->num_of_cam));
	json_object_set_new(options, "mpu_name", json_string(state->mpu_name));
	json_object_set_new(options, "sharpness_gain",
			json_real(state->options.sharpness_gain));
	json_object_set_new(options, "color_offset",
			json_real(state->options.color_offset));
	json_object_set_new(options, "overlap", json_real(state->options.overlap));
	for (int i = 0; i < state->num_of_cam; i++) {
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
		sprintf(buff, "cam%d_aov", i);
		json_object_set_new(options, buff,
				json_real(state->options.cam_aov[i]));
	}
	{ //rtp
		json_object_set_new(options, "rtp_rx_port",
				json_integer(state->options.rtp_rx_port));
		json_object_set_new(options, "rtp_rx_type",
				json_string(
						rtp_get_rtp_socket_type_str(
								state->options.rtp_rx_type)));
		json_object_set_new(options, "rtp_tx_ip",
				json_string(state->options.rtp_tx_ip));
		json_object_set_new(options, "rtp_tx_port",
				json_integer(state->options.rtp_tx_port));
		json_object_set_new(options, "rtp_tx_type",
				json_string(
						rtp_get_rtp_socket_type_str(
								state->options.rtp_tx_type)));
	}
	{ //rtcp
		json_object_set_new(options, "rtcp_rx_port",
				json_integer(state->options.rtcp_rx_port));
		json_object_set_new(options, "rtcp_rx_type",
				json_string(
						rtp_get_rtp_socket_type_str(
								state->options.rtcp_rx_type)));
		json_object_set_new(options, "rtcp_tx_ip",
				json_string(state->options.rtcp_tx_ip));
		json_object_set_new(options, "rtcp_tx_port",
				json_integer(state->options.rtcp_tx_port));
		json_object_set_new(options, "rtcp_tx_type",
				json_string(
						rtp_get_rtp_socket_type_str(
								state->options.rtcp_tx_type)));
	}
	json_object_set_new(options, "is_samplerExternalOES",
			json_integer(state->options.is_samplerExternalOES));

	if (state->plugin_paths) {
		json_t *plugin_paths = json_array();
		for (int i = 0; state->plugin_paths[i] != NULL; i++) {
			json_array_append_new(plugin_paths,
					json_string(state->plugin_paths[i]));
		}
		json_object_set_new(options, "plugin_paths", plugin_paths);
	}

	for (int i = 0; state->plugins[i] != NULL; i++) {
		if (state->plugins[i]->save_options) {
			state->plugins[i]->save_options(state->plugins[i]->user_data,
					options);
		}
	}

	json_dump_file(options, state->config_filepath,
			JSON_PRESERVE_ORDER | JSON_INDENT(4) | JSON_REAL_PRECISION(9));

	json_decref(options);
}

static void init_options_ex(PICAM360CAPTURE_T *state) {
	json_error_t error;
	json_t *options = json_load_file(state->options.config_ex_filepath, 0,
			&error);
	if (options != NULL) {
		for (int i = 0; i < state->num_of_cam; i++) {
			char buff[256];
			sprintf(buff, "cam%d_offset_x", i);
			state->options.cam_offset_x_ex[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_offset_y", i);
			state->options.cam_offset_y_ex[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, "cam%d_horizon_r", i);
			state->options.cam_horizon_r_ex[i] = json_number_value(
					json_object_get(options, buff));
		}
	}
}

static void save_options_ex(PICAM360CAPTURE_T *state) {
	json_t *options = json_object();

	for (int i = 0; i < state->num_of_cam; i++) {
		char buff[256];
		sprintf(buff, "cam%d_offset_x", i);
		json_object_set_new(options, buff,
				json_real(state->options.cam_offset_x_ex[i]));
		sprintf(buff, "cam%d_offset_y", i);
		json_object_set_new(options, buff,
				json_real(state->options.cam_offset_y_ex[i]));
		sprintf(buff, "cam%d_horizon_r", i);
		json_object_set_new(options, buff,
				json_real(state->options.cam_horizon_r_ex[i]));
	}

	json_dump_file(options, state->options.config_ex_filepath, JSON_INDENT(4));

	json_decref(options);
}
//------------------------------------------------------------------------------

static void exit_func(void) {
	printf("\npicam360-capture closed\n");
}

//==============================================================================

static int _command_handler(int argc, char *argv[]) {
	int opt;
	int ret = 0;
	char *cmd = argv[0];
	if (cmd == NULL) {
		//do nothing
	} else if (strcmp(cmd, "exit") == 0 || strcmp(cmd, "q") == 0
			|| strcmp(cmd, "quit") == 0) {
		printf("exit\n");
		exit(0);
	} else if (strcmp(cmd, "save") == 0) {
		save_options(state);
		{ //send upstream
			char cmd[256];
			sprintf(cmd, "upstream.save");
			state->plugin_host.send_command(cmd);
		}
		printf("save config done\n");
	} else if (cmd[1] == '\0' && cmd[0] >= '0' && cmd[0] <= '9') {
		state->active_cam = cmd[0] - '0';
	} else if (strcmp(cmd, "snap") == 0) {
		//TODO
	} else if (strcmp(cmd, "build_vstream") == 0) {
		char *s_str = NULL;
		char uuid_str[37] = { };
		uuid_t uuid = { };
		optind = 1; // reset getopt
		while ((opt = getopt(argc, argv, "u:s:")) != -1) {
			switch (opt) {
			case 'u':
				sscanf(optarg, "%36s", uuid_str);
				uuid_parse(uuid_str, uuid);
				break;
			case 's':
				s_str = optarg;
				break;
			}
		}
		if (s_str != NULL && uuid[0] != 0) {
			VSTREAMER_T *vstreamer = state->plugin_host.build_vstream(uuid,
					s_str);
			if (vstreamer) {
				vstreamer->start(vstreamer);
				printf("%s : complete uuid=%s\n", cmd, uuid_str);
			} else {
				printf("%s : failed uuid=%s\n", cmd, uuid_str);
			}

		}
	} else if (strcmp(cmd, "destroy_vstream") == 0) {
		bool delete_all = false;
		char uuid_str[37] = { };
		uuid_t uuid = { };
		optind = 1; // reset getopt
		while ((opt = getopt(argc, argv, "au:")) != -1) {
			switch (opt) {
			case 'a':
				delete_all = true;
				break;
			case 'u':
				sscanf(optarg, "%36s", uuid_str);
				uuid_parse(uuid_str, uuid);
				break;
			}
		}
		if (delete_all) {
			while (state->vostream_list) {
				VSTREAMER_T *streamer =
						(VSTREAMER_T*) state->vostream_list->value;
				uuid_unparse(streamer->uuid, uuid_str);
				bool ret = state->plugin_host.destroy_vstream(streamer->uuid);
				printf("%s : complete %s : %s\n", cmd, uuid_str,
						ret ? "true" : "false");
			}
			printf("%s all : complete\n", cmd);
		} else if (uuid[0] != 0) {
			bool ret = state->plugin_host.destroy_vstream(uuid);
			printf("%s : complete %s : %s\n", cmd, uuid_str,
					ret ? "true" : "false");
		} else {
			printf("not specified uuid\n");
		}
	} else if (strcmp(cmd, "set_vostream_param") == 0) {
		VSTREAMER_T *streamer = NULL;
		char uuid_str[37] = { };
		uuid_t uuid = { };
		optind = 1; // reset getopt
		while ((opt = getopt(argc, argv, "u:p:")) != -1) {
			switch (opt) {
			case 'u':
				sscanf(optarg, "%36s", uuid_str);
				uuid_parse(uuid_str, uuid);
				break;
			}
		}
		if (uuid_str[0] != 0) {
			LIST_T **pp;
			LIST_FOR_START(pp, VSTREAMER_T, _streamer, state->vostream_list)
				if (uuid_compare(_streamer->uuid, uuid) == 0) {
					streamer = _streamer;
					break;
				}
			}
		}
		if (streamer == NULL) {
			printf("not existing uuid=%s\n", uuid_str);
		}
		optind = 1; // reset getopt
		while ((opt = getopt(argc, argv, "u:p:")) != -1) {
			switch (opt) {
			case 'p':
				if (streamer) {
					int len = strlen(optarg);
					char name[64] = { };
					char value[512] = { };
					for (int i = 0; optarg[i] != '\0'; i++) {
						if (optarg[i] == '=' && i < sizeof(name)
								&& len - (i + 1) < sizeof(value)) {
							memcpy(name, optarg, i);
							memcpy(value, optarg + (i + 1), len - (i + 1));
							streamer->next_streamer->set_param(
									streamer->next_streamer, name, value);
							break;
						}
					}
				}
				break;
			}
		}
	} else if (strcmp(cmd, "set_camera_orientation") == 0) {
		if (argv[1] != NULL) {
			float pitch;
			float yaw;
			float roll;
			sscanf(argv[1], "%f,%f,%f", &pitch, &yaw, &roll);
			state->camera_pitch = pitch * M_PI / 180.0;
			state->camera_yaw = yaw * M_PI / 180.0;
			state->camera_roll = roll * M_PI / 180.0;
			printf("set_camera_orientation\n");
		}
	} else if (strcmp(cmd, "set_conf_sync") == 0) {
		if (argv[1] != NULL) {
			state->conf_sync = (argv[1][0] == '1');
			printf("set_conf_sync %s\n", argv[1]);
		}
	} else if (strcmp(cmd, "set_preview") == 0) {
		if (argv[1] != NULL) {
			state->preview = (argv[1][0] == '1');
			printf("set_preview %s\n", argv[1]);
		}
	} else if (strcmp(cmd, "add_camera_horizon_r") == 0) {
		if (argv[1] != NULL) {
			float *cam_horizon_r =
					(state->options.config_ex_enabled) ?
							state->options.cam_horizon_r_ex :
							state->options.cam_horizon_r;

			int cam_num = 0;
			float value = 0;
			if (argv[1][0] == '*') {
				sscanf(argv[1], "*=%f", &value);
				for (int i = 0; i < state->num_of_cam; i++) {
					cam_horizon_r[i] += value;
				}
			} else {
				sscanf(argv[1], "%d=%f", &cam_num, &value);
				if (cam_num >= 0 && cam_num < state->num_of_cam) {
					cam_horizon_r[cam_num] += value;
				}
			}

			if (state->options.config_ex_enabled) { //try loading configuration
				save_options_ex(state);
			} else { //send upstream
				char cmd[256];
				sprintf(cmd, "upstream.add_camera_horizon_r %s", argv[1]);
				state->plugin_host.send_command(cmd);
			}

			printf("add_camera_horizon_r : completed\n");
		}
	} else if (strcmp(cmd, "add_camera_offset_x") == 0) {
		if (argv[1] != NULL) {
			float *cam_offset_x =
					(state->options.config_ex_enabled) ?
							state->options.cam_offset_x_ex :
							state->options.cam_offset_x;

			int cam_num = 0;
			float value = 0;
			if (argv[1][0] == '*') {
				sscanf(argv[1], "*=%f", &value);
				for (int i = 0; i < state->num_of_cam; i++) {
					cam_offset_x[i] += value;
				}
			} else {
				sscanf(argv[1], "%d=%f", &cam_num, &value);
				if (cam_num >= 0 && cam_num < state->num_of_cam) {
					cam_offset_x[cam_num] += value;
				}
			}

			if (state->options.config_ex_enabled) { //try loading configuration
				save_options_ex(state);
			} else { //send upstream
				char cmd[256];
				sprintf(cmd, "upstream.add_camera_offset_x %s", argv[1]);
				state->plugin_host.send_command(cmd);
			}

			printf("add_camera_offset_x : completed\n");
		}
	} else if (strcmp(cmd, "add_camera_offset_y") == 0) {
		if (argv[1] != NULL) {
			float *cam_offset_y =
					(state->options.config_ex_enabled) ?
							state->options.cam_offset_y_ex :
							state->options.cam_offset_y;

			int cam_num = 0;
			float value = 0;
			if (argv[1][0] == '*') {
				sscanf(argv[1], "*=%f", &value);
				for (int i = 0; i < state->num_of_cam; i++) {
					cam_offset_y[i] += value;
				}
			} else {
				sscanf(argv[1], "%d=%f", &cam_num, &value);
				if (cam_num >= 0 && cam_num < state->num_of_cam) {
					cam_offset_y[cam_num] += value;
				}
			}

			if (state->options.config_ex_enabled) { //try loading configuration
				save_options_ex(state);
			} else { //send upstream
				char cmd[256];
				sprintf(cmd, "upstream.add_camera_offset_y %s", argv[1]);
				state->plugin_host.send_command(cmd);
			}

			printf("add_camera_offset_y : completed\n");
		}
	} else if (strcmp(cmd, "add_camera_offset_yaw") == 0) {
		if (argv[1] != NULL) {
			int cam_num = 0;
			float value = 0;
			if (argv[1][0] == '*') {
				sscanf(argv[1], "*=%f", &value);
				for (int i = 0; i < state->num_of_cam; i++) {
					state->options.cam_offset_yaw[i] += value;
				}
			} else {
				sscanf(argv[1], "%d=%f", &cam_num, &value);
				if (cam_num >= 0 && cam_num < state->num_of_cam) {
					state->options.cam_offset_yaw[cam_num] += value;
				}
			}
			{ //send upstream
				char cmd[256];
				sprintf(cmd, "upstream.add_camera_offset_yaw %s", argv[1]);
				state->plugin_host.send_command(cmd);
			}

			printf("add_camera_offset_yaw : completed\n");
		}
	} else if (strcmp(cmd, "set_camera_horizon_r_bias") == 0) {
		if (argv[1] != NULL) {
			float value = 0;
			sscanf(argv[1], "%f", &value);
			state->camera_horizon_r_bias = value;
			printf("set_camera_horizon_r_bias : completed\n");
		}
	} else if (strcmp(cmd, "set_play_speed") == 0) {
		if (argv[1] != NULL) {
			float value = 0;
			sscanf(argv[1], "%f", &value);
			state->rtp_play_speed = value;
			rtp_set_play_speed(state->rtp, value);
			printf("set_play_speed : completed\n");
		}
	} else if (strcmp(cmd, "add_color_offset") == 0) {
		if (argv[1] != NULL) {
			float value = 0;
			sscanf(argv[1], "%f", &value);
			state->options.color_offset += value;
			printf("add_color_offset : completed\n");
		}
	} else if (strcmp(cmd, "set_frame_sync") == 0) {
		if (argv[1] != NULL) {
			state->frame_sync = (argv[1][0] == '1');
			printf("set_frame_sync %s\n", argv[1]);
		}
	} else if (strcmp(cmd, "set_menu_visible") == 0) {
		if (argv[1] != NULL) {
			state->menu_visible = (argv[1][0] == '1');
			printf("set_menu_visible %s\n", argv[1]);
		}
	} else if (strcmp(cmd, "select_active_menu") == 0) {
		menu_operate(state->menu, MENU_OPERATE_SELECT);
	} else if (strcmp(cmd, "deselect_active_menu") == 0) {
		menu_operate(state->menu, MENU_OPERATE_DESELECT);
	} else if (strcmp(cmd, "go2next_menu") == 0) {
		menu_operate(state->menu, MENU_OPERATE_ACTIVE_NEXT);
	} else if (strcmp(cmd, "back2previouse_menu") == 0) {
		menu_operate(state->menu, MENU_OPERATE_ACTIVE_BACK);
//	} else if (state->frame != NULL && strcasecmp(state->frame->renderer->name, "CALIBRATION") == 0) {
//		static double calib_step = 0.01;
//		if (strcmp(cmd, "step") == 0) {
//			char *param = strtok(NULL, " \n");
//			if (param != NULL) {
//				sscanf(param, "%lf", &calib_step);
//			}
//		}
//		if (strcmp(cmd, "u") == 0 || strcmp(cmd, "t") == 0) {
//			state->options.cam_offset_y[state->active_cam] += calib_step;
//		}
//		if (strcmp(cmd, "d") == 0 || strcmp(cmd, "b") == 0) {
//			state->options.cam_offset_y[state->active_cam] -= calib_step;
//		}
//		if (strcmp(cmd, "l") == 0) {
//			state->options.cam_offset_x[state->active_cam] += calib_step;
//		}
//		if (strcmp(cmd, "r") == 0) {
//			state->options.cam_offset_x[state->active_cam] -= calib_step;
//		}
//		if (strcmp(cmd, "s") == 0) {
//			state->options.sharpness_gain += calib_step;
//		}
//		if (strcmp(cmd, "w") == 0) {
//			state->options.sharpness_gain -= calib_step;
//		}
	} else {
		printf("unknown command : %s\n", cmd);
	}
	return ret;
}

static int command_handler() {
	int ret = 0;

	for (int i = 0; i < 10; i++) {
		char *buff = NULL;
		{
			pthread_mutex_lock(&state->cmd_list_mutex);

			if (state->cmd_list) {
				LIST_T *cur = state->cmd_list;
				buff = (char*) cur->value;
				state->cmd_list = cur->next;
				free(cur);
			}

			pthread_mutex_unlock(&state->cmd_list_mutex);
		}

		if (buff) {
			bool handled = false;
			for (int i = 0; state->plugins[i] != NULL; i++) {
				int name_len = strlen(state->plugins[i]->name);
				if (strncmp(buff, state->plugins[i]->name, name_len) == 0
						&& buff[name_len] == '.') {
					ret = state->plugins[i]->command_handler(
							state->plugins[i]->user_data, buff + name_len + 1);
					handled = true;
				}
			}
			if (!handled) {
				wordexp_t p;
				ret = wordexp(buff, &p, 0);
				if (ret != 0) {
					printf("command parse error : %s", buff);
				} else {
					ret = _command_handler(p.we_wordc, p.we_wordv);
				}
				wordfree(&p);
			}
			free(buff);
		} else {
			break;
		}
	}
	return ret;
}

static void* readline_thread_func(void *arg) {
	char *ptr;
	using_history();
	read_history(PICAM360_HISTORY_FILE);
	while ((ptr = readline("picam360-capture>")) != NULL) {
		add_history(ptr);
		state->plugin_host.send_command(ptr);
		free(ptr);

		write_history(PICAM360_HISTORY_FILE);
	}
	return NULL;
}

static void* quaternion_thread_func(void *arg) {
	int count = 0;
	while (1) {
		if (state->mpu) {
			int cur = (state->quaternion_queue_cur + 1)
					% MAX_QUATERNION_QUEUE_COUNT;
			state->quaternion_queue[cur] = state->mpu->get_quaternion(
					state->mpu);
			state->quaternion_queue_cur++;
		}
		usleep(QUATERNION_QUEUE_RES * 1000);
	}
	return NULL;
}

static void* istream_thread_func(void *arg) {
	int count = 0;
	while (1) {
		if (state->mpu) {
			int cur = (state->quaternion_queue_cur + 1)
					% MAX_QUATERNION_QUEUE_COUNT;
			state->quaternion_queue[cur] = state->mpu->get_quaternion(
					state->mpu);
			state->quaternion_queue_cur++;
		}
		usleep(QUATERNION_QUEUE_RES * 1000);
	}
	return NULL;
}

///////////////////////////////////////////
#if (1) //plugin host methods
static VECTOR4D_T get_view_quaternion() {
	VECTOR4D_T ret = { };
//	if (state->frame && state->frame->view_mpu) {
//		ret = state->frame->view_mpu->get_quaternion(state->frame->view_mpu);
//	}
	return ret;
}
static VECTOR4D_T get_view_compass() {
	VECTOR4D_T ret = { };
//	if (state->frame && state->frame->view_mpu) {
//		ret = state->frame->view_mpu->get_compass(state->frame->view_mpu);
//	}
	return ret;
}
static float get_view_temperature() {
//	if (state->frame && state->frame->view_mpu) {
//		return state->frame->view_mpu->get_temperature(state->frame->view_mpu);
//	}
	return 0;
}
static float get_view_north() {
//	if (state->frame && state->frame->view_mpu) {
//		return state->frame->view_mpu->get_north(state->frame->view_mpu);
//	}
	return 0;
}

static int get_number_of_cameras() {
	return state->num_of_cam;
}
static VECTOR4D_T get_camera_offset(int cam_num) {
	VECTOR4D_T ret = { };
	pthread_mutex_lock(&state->mutex);

	if (cam_num >= 0 && cam_num < state->num_of_cam) {
		ret.x = state->options.cam_offset_x[cam_num];
		ret.y = state->options.cam_offset_y[cam_num];
		ret.z = state->options.cam_offset_yaw[cam_num];
		ret.w = state->options.cam_horizon_r[cam_num];
	}

	pthread_mutex_unlock(&state->mutex);
	return ret;
}
static void set_camera_offset(int cam_num, VECTOR4D_T value) {
	if (!state->conf_sync) {
		return;
	}

	pthread_mutex_lock(&state->mutex);

	if (cam_num >= 0 && cam_num < state->num_of_cam) {
		state->options.cam_offset_x[cam_num] = value.x;
		state->options.cam_offset_y[cam_num] = value.y;
		state->options.cam_offset_yaw[cam_num] = value.z;
		state->options.cam_horizon_r[cam_num] = value.w;
	}

	pthread_mutex_unlock(&state->mutex);
}

static VECTOR4D_T get_camera_quaternion(int cam_num) {
	VECTOR4D_T ret = { };
	pthread_mutex_lock(&state->mutex);

	if (cam_num >= 0 && cam_num < state->num_of_cam) {
		ret = state->camera_quaternion[cam_num];
	} else {
		ret = state->camera_quaternion[MAX_CAM_NUM];
	}

	pthread_mutex_unlock(&state->mutex);
	return ret;
}
static void set_camera_quaternion(int cam_num, VECTOR4D_T value) {
	pthread_mutex_lock(&state->mutex);

	if (cam_num >= 0 && cam_num < state->num_of_cam) {
		state->camera_quaternion[cam_num] = value;
	}
	state->camera_quaternion[MAX_CAM_NUM] = value; //latest
	state->camera_coordinate_from_device = true;

	pthread_mutex_unlock(&state->mutex);
}
static VECTOR4D_T get_camera_compass() {
	VECTOR4D_T ret = { };
	pthread_mutex_lock(&state->mutex);

	ret = state->camera_compass;

	pthread_mutex_unlock(&state->mutex);
	return ret;
}
static void set_camera_compass(VECTOR4D_T value) {
	pthread_mutex_lock(&state->mutex);

	state->camera_compass = value;

	pthread_mutex_unlock(&state->mutex);
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

static void* get_display() {
#ifdef USE_GLES
	return state->egl_handler.display;
#else
	return NULL;
#endif
}
static void lock_texture() {
	pthread_mutex_lock(&state->texture_mutex);
	eh_activate_context(&state->egl_handler);
}
static void unlock_texture() {
	eh_deactivate_context(&state->egl_handler);
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
static void set_texture_size(uint32_t width, uint32_t height) {
	pthread_mutex_lock(&state->texture_size_mutex);
	if (state->cam_width == width && state->cam_height == height) {
		//do nothing
	} else {
		state->cam_width = width;
		state->cam_height = height;
	}
	pthread_mutex_unlock(&state->texture_size_mutex);
}
static MENU_T* get_menu() {
	return state->menu;
}
static bool get_menu_visible() {
	return state->menu_visible;
}
static void set_menu_visible(bool value) {
	state->menu_visible = value;
}

static float get_fov() {
	//return state->frame->fov;
	return 0;
}
static void set_fov(float value) {
	//state->frame->fov = value;
}

static MPU_T* get_mpu() {
	return state->mpu;
}
static RTP_T* get_rtp() {
	return state->rtp;
}
static RTP_T* get_rtcp() {
	return state->rtcp;
}

static int xmp(char *buff, int buff_len, int cam_num) {
	int xmp_len = 0;

	struct timeval timestamp = { };
	gettimeofday(&timestamp, NULL);

	VECTOR4D_T quat = { };
	{
		int video_delay_ms = 0;
		int cur = (state->quaternion_queue_cur
				- video_delay_ms / QUATERNION_QUEUE_RES
				+ MAX_QUATERNION_QUEUE_COUNT) % MAX_QUATERNION_QUEUE_COUNT;
		quat = state->quaternion_queue[cur];
	}
	VECTOR4D_T compass = state->mpu->get_compass(state->mpu);
	VECTOR4D_T camera_offset = state->plugin_host.get_camera_offset(cam_num);

	xmp_len = 0;
	buff[xmp_len++] = 0xFF;
	buff[xmp_len++] = 0xE1;
	buff[xmp_len++] = 0; // size MSB
	buff[xmp_len++] = 0; // size LSB
	xmp_len += sprintf(buff + xmp_len, "http://ns.adobe.com/xap/1.0/");
	buff[xmp_len++] = '\0';
	xmp_len += sprintf(buff + xmp_len, "<?xpacket begin=\"﻿");
	buff[xmp_len++] = 0xEF;
	buff[xmp_len++] = 0xBB;
	buff[xmp_len++] = 0xBF;
	xmp_len += sprintf(buff + xmp_len, "\" id=\"W5M0MpCehiHzreSzNTczkc9d\"?>");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<x:xmpmeta xmlns:x=\"adobe:ns:meta/\" x:xmptk=\"picam360-capture rev1\">");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">");
	xmp_len += sprintf(buff + xmp_len, "<rdf:Description rdf:about=\"\">");
	xmp_len += sprintf(buff + xmp_len,
			"<quaternion x=\"%f\" y=\"%f\" z=\"%f\" w=\"%f\" />", quat.x,
			quat.y, quat.z, quat.w);
	xmp_len += sprintf(buff + xmp_len, "<compass x=\"%f\" y=\"%f\" z=\"%f\" />",
			compass.x, compass.y, compass.z);
	xmp_len += sprintf(buff + xmp_len, "<temperature v=\"%f\" />",
			state->mpu->get_temperature(state->mpu));
	xmp_len += sprintf(buff + xmp_len,
			"<offset x=\"%f\" y=\"%f\" yaw=\"%f\" horizon_r=\"%f\" />",
			camera_offset.x, camera_offset.y, camera_offset.z, camera_offset.w);
	xmp_len += sprintf(buff + xmp_len, "<timestamp sec=\"%lu\" usec=\"%lu\" />",
			(uint64_t) timestamp.tv_sec, (uint64_t) timestamp.tv_usec);
	xmp_len += sprintf(buff + xmp_len, "</rdf:Description>");
	xmp_len += sprintf(buff + xmp_len, "</rdf:RDF>");
	xmp_len += sprintf(buff + xmp_len, "</x:xmpmeta>");
	xmp_len += sprintf(buff + xmp_len, "<?xpacket end=\"w\"?>");
	buff[xmp_len++] = '\0';
	buff[2] = ((xmp_len - 2) >> 8) & 0xFF; // size MSB
	buff[3] = (xmp_len - 2) & 0xFF; // size LSB

	return xmp_len;
}

static void send_command(const char *_cmd) {
	pthread_mutex_lock(&state->cmd_list_mutex);

	char *cmd = (char*) _cmd;
	LIST_T **cur = NULL;
	if (strncmp(cmd, ENDPOINT_DOMAIN, ENDPOINT_DOMAIN_SIZE) == 0) {
		if (state->options.rtp_rx_port == 0) {
			//endpoint
			cur = &state->cmd_list;
			cmd += ENDPOINT_DOMAIN_SIZE;
		} else {
			//send to upstream
			cur = &state->cmd2upstream_list;
		}
	} else if (strncmp(cmd, UPSTREAM_DOMAIN, UPSTREAM_DOMAIN_SIZE) == 0) {
		cur = &state->cmd2upstream_list;
		cmd += UPSTREAM_DOMAIN_SIZE;
	} else {
		cur = &state->cmd_list;
	}
	for (; *cur != NULL; cur = &(*cur)->next)
		;
	*cur = malloc(sizeof(LIST_T));
	memset(*cur, 0, sizeof(LIST_T));
	int slr_len = MIN(strlen(cmd), 256);
	char *cmd_clone = malloc(slr_len + 1);
	strncpy(cmd_clone, cmd, slr_len);
	cmd_clone[slr_len] = '\0';
	(*cur)->value = cmd_clone;

	pthread_mutex_unlock(&state->cmd_list_mutex);
}

static void event_handler(uint32_t node_id, uint32_t event_id) {
	switch (node_id) {
	case PICAM360_HOST_NODE_ID:
		switch (event_id) {
		case PICAM360_CAPTURE_EVENT_AFTER_SNAP:
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void send_event(uint32_t node_id, uint32_t event_id) {
	event_handler(node_id, event_id);
	for (int i = 0; state->plugins && state->plugins[i] != NULL; i++) {
		if (state->plugins[i]) {
			state->plugins[i]->event_handler(state->plugins[i]->user_data,
					node_id, event_id);
		}
	}
}

static void add_mpu_factory(MPU_FACTORY_T *mpu_factory) {
	for (int i = 0; state->mpu_factories[i] != (void*) -1; i++) {
		if (state->mpu_factories[i] == NULL) {
			state->mpu_factories[i] = mpu_factory;
			if (state->mpu_factories[i + 1] == (void*) -1) {
				int space = (i + 2) * 2;
				if (space > 256) {
					fprintf(stderr, "error on add_mpu_factory\n");
					return;
				}
				MPU_FACTORY_T **current = state->mpu_factories;
				state->mpu_factories = malloc(sizeof(MPU_FACTORY_T*) * space);
				memset(state->mpu_factories, 0, sizeof(MPU_FACTORY_T*) * space);
				memcpy(state->mpu_factories, current,
						sizeof(MPU_FACTORY_T*) * i);
				state->mpu_factories[space - 1] = (void*) -1;
				free(current);
			}
			return;
		}
	}
}

static void add_vstreamer_factory(VSTREAMER_FACTORY_T *factory) {
	for (int i = 0; state->vstreamer_factories[i] != (void*) -1; i++) {
		if (state->vstreamer_factories[i] == NULL) {
			state->vstreamer_factories[i] = factory;
			if (state->vstreamer_factories[i + 1] == (void*) -1) {
				int space = (i + 2) * 2;
				if (space > 256) {
					fprintf(stderr, "error on add_vstreamer_factory\n");
					return;
				}
				VSTREAMER_FACTORY_T **current = state->vstreamer_factories;
				state->vstreamer_factories = malloc(
						sizeof(VSTREAMER_FACTORY_T*) * space);
				memset(state->vstreamer_factories, 0,
						sizeof(VSTREAMER_FACTORY_T*) * space);
				memcpy(state->vstreamer_factories, current,
						sizeof(VSTREAMER_FACTORY_T*) * i);
				state->vstreamer_factories[space - 1] = (void*) -1;
				free(current);
			}
			return;
		}
	}
}

static void add_status(STATUS_T *status) {
	for (int i = 0; state->statuses[i] != (void*) -1; i++) {
		if (state->statuses[i] == NULL) {
			state->statuses[i] = status;
			if (state->statuses[i + 1] == (void*) -1) {
				int space = (i + 2) * 2;
				if (space > 256) {
					fprintf(stderr, "error on add_status\n");
					return;
				}
				STATUS_T **current = state->statuses;
				state->statuses = malloc(sizeof(STATUS_T*) * space);
				memset(state->statuses, 0, sizeof(STATUS_T*) * space);
				memcpy(state->statuses, current, sizeof(STATUS_T*) * i);
				state->statuses[space - 1] = (void*) -1;
				free(current);
			}
			return;
		}
	}
}

static void add_watch(STATUS_T *watch) {
	for (int i = 0; state->watches[i] != (void*) -1; i++) {
		if (state->watches[i] == NULL) {
			state->watches[i] = watch;
			if (state->watches[i + 1] == (void*) -1) {
				int space = (i + 2) * 2;
				if (space > 256) {
					fprintf(stderr, "error on add_watch\n");
					return;
				}
				STATUS_T **current = state->watches;
				state->watches = malloc(sizeof(STATUS_T*) * space);
				memset(state->watches, 0, sizeof(STATUS_T*) * space);
				memcpy(state->watches, current, sizeof(STATUS_T*) * i);
				state->watches[space - 1] = (void*) -1;
				free(current);
			}
			return;
		}
	}
}

static void add_plugin(PLUGIN_T *plugin) {
	for (int i = 0; state->plugins[i] != (void*) -1; i++) {
		if (state->plugins[i] == NULL) {
			state->plugins[i] = plugin;
			if (state->plugins[i + 1] == (void*) -1) {
				int space = (i + 2) * 2;
				if (space > 256) {
					fprintf(stderr, "error on add_plugin\n");
					return;
				}
				PLUGIN_T **current = state->plugins;
				state->plugins = malloc(sizeof(PLUGIN_T*) * space);
				memset(state->plugins, 0, sizeof(PLUGIN_T*) * space);
				memcpy(state->plugins, current, sizeof(PLUGIN_T*) * i);
				state->plugins[space - 1] = (void*) -1;
				free(current);
			}
			return;
		}
	}
}

static void snap(uint32_t width, uint32_t height, enum RENDERING_MODE mode,
		const char *path) {
	char cmd[512];
	char *mode_str = "";
	switch (mode) {
	case RENDERING_MODE_EQUIRECTANGULAR:
		mode_str = "-E";
		break;
	default:
		break;
	}

	sprintf(cmd, "snap -w %d -h %d %s -o %s", width, height, mode_str, path);
	state->plugin_host.send_command(cmd);
}

static void init_plugin_host(PICAM360CAPTURE_T *state) {
	{ //init host
		state->plugin_host.get_view_quaternion = get_view_quaternion;
		state->plugin_host.get_view_compass = get_view_compass;
		state->plugin_host.get_view_temperature = get_view_temperature;
		state->plugin_host.get_view_north = get_view_north;

		state->plugin_host.get_number_of_cameras = get_number_of_cameras;
		state->plugin_host.get_camera_offset = get_camera_offset;
		state->plugin_host.set_camera_offset = set_camera_offset;
		state->plugin_host.get_camera_quaternion = get_camera_quaternion;
		state->plugin_host.set_camera_quaternion = set_camera_quaternion;
		state->plugin_host.get_camera_compass = get_camera_compass;
		state->plugin_host.set_camera_compass = set_camera_compass;
		state->plugin_host.get_camera_temperature = get_camera_temperature;
		state->plugin_host.set_camera_temperature = set_camera_temperature;
		state->plugin_host.get_camera_north = get_camera_north;
		state->plugin_host.set_camera_north = set_camera_north;

		state->plugin_host.get_display = get_display;
		state->plugin_host.lock_texture = lock_texture;
		state->plugin_host.unlock_texture = unlock_texture;
		state->plugin_host.set_cam_texture_cur = set_cam_texture_cur;
		state->plugin_host.get_texture_size = get_texture_size;
		state->plugin_host.set_texture_size = set_texture_size;
		state->plugin_host.load_texture = load_texture;
		state->plugin_host.get_logo_image = get_logo_image;
		state->plugin_host.get_rendering_params = get_rendering_params;

		state->plugin_host.get_menu = get_menu;
		state->plugin_host.get_menu_visible = get_menu_visible;
		state->plugin_host.set_menu_visible = set_menu_visible;

		state->plugin_host.get_fov = get_fov;
		state->plugin_host.set_fov = set_fov;

		state->plugin_host.get_mpu = get_mpu;
		state->plugin_host.get_rtp = get_rtp;
		state->plugin_host.get_rtcp = get_rtcp;
		state->plugin_host.xmp = xmp;

		state->plugin_host.build_vstream = build_vstream;
		state->plugin_host.destroy_vstream = destroy_vstream;

		state->plugin_host.send_command = send_command;
		state->plugin_host.send_event = send_event;
		state->plugin_host.add_mpu_factory = add_mpu_factory;
		state->plugin_host.add_vstreamer_factory = add_vstreamer_factory;
		state->plugin_host.add_status = add_status;
		state->plugin_host.add_watch = add_watch;
		state->plugin_host.add_plugin = add_plugin;

		state->plugin_host.snap = snap;
	}

	{
		MPU_FACTORY_T *mpu_factory = NULL;
		create_manual_mpu_factory(&mpu_factory);
		state->plugin_host.add_mpu_factory(mpu_factory);
	}
}
static void init_plugins(PICAM360CAPTURE_T *state) {
	//load plugins
	json_error_t error;
	json_t *options = json_load_file_without_comment(state->config_filepath, 0,
			&error);
	if (options == NULL) {
		fputs(error.text, stderr);
	} else {
		{
			json_t *plugin_paths = json_object_get(options, "plugin_paths");
			if (json_is_array(plugin_paths)) {
				int size = json_array_size(plugin_paths);
				state->plugin_paths = (char**) malloc(
						sizeof(char*) * (size + 1));
				memset(state->plugin_paths, 0, sizeof(char*) * (size + 1));

				for (int i = 0; i < size; i++) {
					json_t *value = json_array_get(plugin_paths, i);
					int len = json_string_length(value);
					state->plugin_paths[i] = (char*) malloc(
							sizeof(char) * (len + 1));
					memset(state->plugin_paths[i], 0, sizeof(char) * (len + 1));
					strncpy(state->plugin_paths[i], json_string_value(value),
							len);
					if (len > 0) {
						void *handle = dlopen(state->plugin_paths[i],
								RTLD_LAZY);
						if (!handle) {
							fprintf(stderr, "%s\n", dlerror());
							continue;
						}
						CREATE_PLUGIN create_plugin = (CREATE_PLUGIN) dlsym(
								handle, "create_plugin");
						if (!create_plugin) {
							fprintf(stderr, "%s\n", dlerror());
							dlclose(handle);
							continue;
						}
						PLUGIN_T *plugin = NULL;
						create_plugin(&state->plugin_host, &plugin);
						if (!plugin) {
							fprintf(stderr, "%s\n", "create_plugin fail.");
							dlclose(handle);
							continue;
						}
						printf("plugin %s loaded.\n", state->plugin_paths[i]);
						state->plugin_host.add_plugin(plugin);
					}
				}
			}
		}

		for (int i = 0; state->plugins[i] != NULL; i++) {
			if (state->plugins[i]->init_options) {
				state->plugins[i]->init_options(state->plugins[i]->user_data,
						options);
			}
		}

		json_decref(options);
	}
}

#endif //plugin block

///////////////////////////////////////////////////////
#if (1) //rtp block

static int command2upstream_handler() {
	static struct timeval last_try = { };
	static bool is_first_try = false;
	int len = strlen(state->command);
	if (len != 0 && state->command_id != state->ack_command_id_upstream) {
		struct timeval s;
		gettimeofday(&s, NULL);
		if (!is_first_try) {
			struct timeval diff;
			timersub(&s, &last_try, &diff);
			float diff_sec = (float) diff.tv_sec
					+ (float) diff.tv_usec / 1000000;
			if (diff_sec < 0.050) {
				return 0;
			}
		}
		rtp_sendpacket(state->rtcp, (unsigned char*) state->command, len,
		PT_CMD);
		rtp_flush(state->rtcp);
		last_try = s;
		is_first_try = false;
		return 0;
	} else {
		memset(state->command, 0, sizeof(state->command));
	}

	char *buff = NULL;
	{
		pthread_mutex_lock(&state->cmd_list_mutex);

		if (state->cmd2upstream_list) {
			LIST_T *cur = state->cmd2upstream_list;
			buff = (char*) cur->value;
			state->cmd2upstream_list = cur->next;
			free(cur);
		}

		pthread_mutex_unlock(&state->cmd_list_mutex);
	}

	if (buff) {
		if (state->command_id >= INT_MAX) {
			state->command_id = 0;
		}
		++state->command_id; //command_id shoud start from 1
		snprintf(state->command, sizeof(state->command),
				"<picam360:command id=\"%d\" value=\"%s\" />",
				state->command_id, buff);
		free(buff);
	}
	return 0;
}

static void parse_status(char *data, int data_len) {
	for (int i = 0; i < data_len; i++) {
		if (data[i] == '<') {
			char name[64] = UPSTREAM_DOMAIN;
			char value[256];
			int num = sscanf(&data[i],
					"<picam360:status name=\"%63[^\"]\" value=\"%255[^\"]\" />",
					name + UPSTREAM_DOMAIN_SIZE, value);
			if (num == 2) {
				for (int i = 0; state->watches[i] != NULL; i++) {
					if (strncmp(state->watches[i]->name, name, 64) == 0) {
						state->watches[i]->set_value(
								state->watches[i]->user_data, value);
						break;
					}
				}
			}
		}
	}
}

static int rtp_callback(unsigned char *data, unsigned int data_len,
		unsigned char pt, unsigned int seq_num, void *user_data) {
	if (data_len == 0) {
		return -1;
	}
	static unsigned int last_seq_num = 0;
	if (seq_num != last_seq_num + 1) {
		printf("rtp : packet lost : from %d to %d\n", last_seq_num, seq_num);
	}
	last_seq_num = seq_num;

	if (pt == PT_STATUS) {
		parse_status((char*) data, data_len);
	}
	return 0;
}

static int rtcp_callback(unsigned char *data, unsigned int data_len,
		unsigned char pt, unsigned int seq_num) {
	if (data_len == 0) {
		return -1;
	}
	static unsigned int last_seq_num = -1;
	if (seq_num != last_seq_num + 1) {
		printf("rtcp : packet lost : from %d to %d\n", last_seq_num, seq_num);
	}
	last_seq_num = seq_num;

	if (pt == PT_CMD) {
		int num = 0;
		int id;
		char name[512];
		char value[512];
		int name_s = -1;
		int value_s = -1;
		for (int i = 0; i < data_len; i++) {
			if (value_s < 0 && data[i] == ' ') {
				name_s = i + 1;
			} else if (name_s > 0) {
				if (data[i] == '=') {
					name[i - name_s] = '\0';
					name_s = -1;
				} else {
					name[i - name_s] = data[i];
				}
			}

			if (data[i] == '"' && data[i - 1] == '=') {
				value_s = i + 1;
			} else if (value_s > 0) {
				if (data[i] == '"' && data[i - 1] != '\\') {
					value[i - value_s] = '\0';
					value_s = -1;
					if (strcmp(name, "id") == 0) {
						num += sscanf(value, "%d", &id);
					} else if (strcmp(name, "value") == 0) {
						strchg(value, "\\\"", "\"");
						num++;
						break;
					}
				} else {
					value[i - value_s] = data[i];
				}
			}
		}
		if (num == 2 && id != state->ack_command_id_downstream) {
			state->ack_command_id_downstream = id;
			state->plugin_host.send_command(value);
		}
	}
	return 0;
}

static void init_rtp(PICAM360CAPTURE_T *state) {
	state->rtp = create_rtp(state->options.rtp_rx_port,
			state->options.rtp_rx_type, state->options.rtp_tx_ip,
			state->options.rtp_tx_port, state->options.rtp_tx_type, 0);
	rtp_add_callback(state->rtp, (RTP_CALLBACK) rtp_callback, NULL);

	state->rtcp = create_rtp(state->options.rtcp_rx_port,
			state->options.rtcp_rx_type, state->options.rtcp_tx_ip,
			state->options.rtcp_tx_port, state->options.rtcp_tx_type, 0);
	rtp_add_callback(state->rtcp, (RTP_CALLBACK) rtcp_callback, NULL);
}

#endif //rtp block

///////////////////////////////////////////////////////

int main(int argc, char *argv[]) {

	bool input_file_mode = false;
	int opt;
	char frame_param[256] = { };

	// Clear application state
	const int INITIAL_SPACE = 16;
	memset(state, 0, sizeof(*state));
	uuid_generate(state->uuid);
	state->cam_width = 2048;
	state->cam_height = 2048;
	state->num_of_cam = 1;
	state->preview = false;
	strncpy(state->mpu_name, "manual", sizeof(state->mpu_name));
	state->video_direct = false;
	state->input_mode = INPUT_MODE_CAM;
	state->output_raw = false;
	state->conf_sync = true;
	state->camera_horizon_r_bias = 1.0;
	state->refraction = 1.0;
	state->rtp_play_speed = 1.0;
	strncpy(state->default_view_coordinate_mode, "manual",
			sizeof(state->default_view_coordinate_mode));
	strncpy(state->config_filepath, "config.json",
			sizeof(state->config_filepath));

	{
		state->plugins = malloc(sizeof(PLUGIN_T*) * INITIAL_SPACE);
		memset(state->plugins, 0, sizeof(PLUGIN_T*) * INITIAL_SPACE);
		state->plugins[INITIAL_SPACE - 1] = (void*) -1;
	}
	{
		state->mpu_factories = malloc(sizeof(MPU_FACTORY_T*) * INITIAL_SPACE);
		memset(state->mpu_factories, 0, sizeof(MPU_FACTORY_T*) * INITIAL_SPACE);
		state->mpu_factories[INITIAL_SPACE - 1] = (void*) -1;
	}
	{
		state->vstreamer_factories = malloc(
				sizeof(VSTREAMER_FACTORY_T*) * INITIAL_SPACE);
		memset(state->vstreamer_factories, 0,
				sizeof(VSTREAMER_FACTORY_T*) * INITIAL_SPACE);
		state->vstreamer_factories[INITIAL_SPACE - 1] = (void*) -1;
	}
	{
		state->watches = malloc(sizeof(STATUS_T*) * INITIAL_SPACE);
		memset(state->watches, 0, sizeof(STATUS_T*) * INITIAL_SPACE);
		state->watches[INITIAL_SPACE - 1] = (void*) -1;
	}
	{
		state->statuses = malloc(sizeof(STATUS_T*) * INITIAL_SPACE);
		memset(state->statuses, 0, sizeof(STATUS_T*) * INITIAL_SPACE);
		state->statuses[INITIAL_SPACE - 1] = (void*) -1;
	}

	umask(0000);

	optind = 1; // reset getopt
	while ((opt = getopt(argc, argv, "c:psi:r:F:v:")) != -1) {
		switch (opt) {
		case 'c':
			strncpy(state->config_filepath, optarg,
					sizeof(state->config_filepath));
			break;
		case 'p':
			state->preview = true;
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
			strncpy(state->default_view_coordinate_mode, optarg, 64);
			break;
		default:
			/* '?' */
			printf("Usage: %s [-c conf_filepath] [-p] [-s]\n", argv[0]);
			return -1;
		}
	}

	//init options
	init_options(state);

	{ //mrevent & mutex init
		for (int i = 0; i < state->num_of_cam; i++) {
			mrevent_init(&state->request_frame_event[i]);
			mrevent_trigger(&state->request_frame_event[i]);
			mrevent_init(&state->arrived_frame_event[i]);
			mrevent_reset(&state->arrived_frame_event[i]);
		}

		pthread_mutex_init(&state->mutex, 0);
		//texture mutex init
		pthread_mutex_init(&state->texture_mutex, 0);
		//texture size mutex init
		pthread_mutex_init(&state->texture_size_mutex, 0);
		//frame mutex init
		pthread_mutex_init(&state->cmd_list_mutex, 0);
	}
#if BCM_HOST
	bcm_host_init();
	printf("Note: ensure you have sufficient gpu_mem configured\n");
#endif

	init_plugin_host(state);

	// init EGL
	init_egl(&state->egl_handler);

	//init rtp
	init_rtp(state);

	//menu
	init_menu_handler(state);

	//status handling
	init_status_handler(state);

	state->plugin_host.lock_texture();
	{
		// Setup the model world
		//TODO init_model_proj(state);
		// initialise the OGLES texture(s)
		//TODO init_textures(state);
		// init plugin
		init_plugins(state);
	}
	state->plugin_host.unlock_texture();

	//frame id=0
	if (frame_param[0]) {
		char cmd[256];
		sprintf(cmd, "build_vstream %s", frame_param);
		state->plugin_host.send_command(cmd);
	}
	//set mpu
	for (int i = 0; state->mpu_factories[i] != NULL; i++) {
		if (strncmp(state->mpu_factories[i]->name, state->mpu_name, 64) == 0) {
			state->mpu_factories[i]->create_mpu(
					state->mpu_factories[i]->user_data, &state->mpu);
		}
	}
	if (state->mpu == NULL) {
		printf("something wrong with %s\n", state->mpu_name);
	}

	static struct timeval last_time = { };
	gettimeofday(&last_time, NULL);

	static struct timeval last_statuses_handled_time = { };
	gettimeofday(&last_statuses_handled_time, NULL);

	//readline
	pthread_t readline_thread;
	pthread_create(&readline_thread, NULL, readline_thread_func, (void*) NULL);

	//quaternion
	pthread_t quaternion_thread;
	pthread_create(&quaternion_thread, NULL, quaternion_thread_func,
			(void*) NULL);

	//status buffer
	char *status_value = (char*) malloc(RTP_MAXPAYLOADSIZE);
	char *status_buffer = (char*) malloc(RTP_MAXPAYLOADSIZE);
	char *status_packet = (char*) malloc(RTP_MAXPAYLOADSIZE);

	while (!terminate) {
		struct timeval time = { };
		gettimeofday(&time, NULL);

		if (state->frame_sync) {
			int res = 0;
			for (int i = 0; i < state->num_of_cam; i++) {
				int res = mrevent_wait(&state->arrived_frame_event[i], 10000); //wait 1msec
				if (res != 0) {
					break;
				}
			}
			if (res != 0) {
				continue; // skip
			}
		}
		//TODO frame_handler();
		command_handler();
		command2upstream_handler();
//		if (state->frame) {
//			for (int i = 0; i < state->num_of_cam; i++) {
//				mrevent_reset(&state->arrived_frame_event[i]);
//				mrevent_trigger(&state->request_frame_event[i]);
//			}
//		}
		if (input_file_mode
				&& state->input_file_cur == state->input_file_size) {
			terminate = true;
		}

		{ //wait 10msec at least for performance
			struct timeval diff;
			timersub(&time, &last_time, &diff);
			float diff_sec = (float) diff.tv_sec
					+ (float) diff.tv_usec / 1000000;
			if (diff_sec < 0.010) { //10msec
				int delay_ms = 10 - (int) (diff_sec * 1000);
				usleep(delay_ms * 1000);
			}
		}
		{ //status
			struct timeval diff;
			timersub(&time, &last_statuses_handled_time, &diff);
			float diff_sec = (float) diff.tv_sec
					+ (float) diff.tv_usec / 1000000;
			if (diff_sec > 0.100) { //less than 10Hz
				int cur = 0;
				for (int i = 0; state->statuses[i]; i++) {
					state->statuses[i]->get_value(state->statuses[i]->user_data,
							status_value, RTP_MAXPAYLOADSIZE);
					int len = snprintf(status_buffer, RTP_MAXPAYLOADSIZE,
							"<picam360:status name=\"%s\" value=\"%s\" />",
							state->statuses[i]->name, status_value);
					if (len >= RTP_MAXPAYLOADSIZE) {
						continue;
					}
					if (cur != 0 && cur + len > RTP_MAXPAYLOADSIZE) {
						rtp_sendpacket(state->rtp,
								(unsigned char*) status_packet, cur, PT_STATUS);
						cur = 0;
					}
					strncpy(status_packet + cur, status_buffer, len);
					cur += len;
				}
				if (cur != 0) {
					rtp_sendpacket(state->rtp, (unsigned char*) status_packet,
							cur, PT_STATUS);
				}
				rtp_flush(state->rtp);

				last_statuses_handled_time = time;
			}
		}
		last_time = time;
	}
	exit_func();
	return 0;
}

