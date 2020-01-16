#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <limits.h>
#include <sys/time.h>
#include <list>
#include <dlfcn.h>
#include <assert.h>
#include <linux/videodev2.h>
#include <wordexp.h>

#include "fisheye_params.h"
#include "mrevent.h"

#define PLUGIN_NAME "fisheye_params"
#define VSTREAMER_NAME "fisheye_params"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static PLUGIN_HOST_T *lg_plugin_host = NULL;

typedef struct _FISHEYE_PARAMS_T {
	float cam_c[2];
	float cam_f[2];
	float lens_k[4];

	float cam_offset[3]; // x,y,z axis rotation
} FISHEYE_PARAMS_T;

typedef struct _plugin_private {
	PLUGIN_T super;
	FISHEYE_PARAMS_T fisheye_params[MAX_CAM_NUM];
} plugin_private;
static plugin_private *lg_plugin = NULL;

typedef struct _PICAM360_IMAGE_WRAPPER_T {
	PICAM360_IMAGE_T super;
	REFERENCE_H *original_ref;
} PICAM360_IMAGE_WRAPPER_T;
typedef struct _fisheye_params_private {
	VSTREAMER_T super;

	int cam_num;
} fisheye_params_private;

static void start(void *user_data) {
	fisheye_params_private *_this = (fisheye_params_private*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}
}

static void stop(void *user_data) {
	fisheye_params_private *_this = (fisheye_params_private*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}
static int get_frame_info_str(FISHEYE_PARAMS_T *params, char *buff,
		int *buff_sizse) {
	int len = 0;

	len += sprintf(buff + len, "<picam360:frame ");
	len += sprintf(buff + len, "mode=\"%s\" ", "FISHEYE");
	len += sprintf(buff + len, "cam_c=\"%.3f,%.3f\" ", params->cam_c[0],
			params->cam_c[1]);
	len += sprintf(buff + len, "cam_f=\"%.3f,%.3f\" ", params->cam_f[0],
			params->cam_f[1]);
	len += sprintf(buff + len, "lens_k=\"%.3f,%.3f,%.3f,%.3f\" ",
			params->lens_k[0], params->lens_k[1], params->lens_k[2],
			params->lens_k[3]);
	len += sprintf(buff + len, "cam_offset=\"%.3f,%.3f,%.3f\" ",
			params->cam_offset[0], params->cam_offset[1],
			params->cam_offset[2]);
	len += sprintf(buff + len, "/>");

	*buff_sizse = len;

	return 0;
}

static int release_PICAM360_IMAGE_WRAPPER_T(void *obj) {
	PICAM360_IMAGE_WRAPPER_T *image = (PICAM360_IMAGE_WRAPPER_T*) obj;

	if (image->super.meta) {
		free(image->super.meta);
	}
	if (image->original_ref) {
		image->original_ref->release(image->original_ref);
	}
	free(image);

	return 0;
}

static int get_image(void *obj, PICAM360_IMAGE_T **image_p, int *num_p,
		int wait_usec) {
	fisheye_params_private *_this = (fisheye_params_private*) obj;

	int num = MAX_CAM_NUM;
	PICAM360_IMAGE_T *images[MAX_CAM_NUM];
	int ret = _this->super.pre_streamer->get_image(_this->super.pre_streamer,
			images, &num, 100 * 1000);
	if (ret != 0) {
		return ret;
	}

	*num_p = MIN(num, *num_p);
	for (int i = 0; i < *num_p; i++) {
		PICAM360_IMAGE_WRAPPER_T *image = (PICAM360_IMAGE_WRAPPER_T*) malloc(
				sizeof(PICAM360_IMAGE_WRAPPER_T));
		image->super = *images[i];
		image->original_ref = images[i]->ref;
		image->super.meta = (unsigned char*) malloc(RTP_MAXPAYLOADSIZE);
		create_reference(&image->super.ref, release_PICAM360_IMAGE_WRAPPER_T,
				image);

		get_frame_info_str(&lg_plugin->fisheye_params[i],
				(char*) image->super.meta, (int*) &image->super.meta_size);
		image_p[i] = &image->super;
	}

	return 0;
}

static void release(void *obj) {
	free(obj);
}

static int set_param(void *obj, const char *param, const char *value_str) {
	fisheye_params_private *_this = (fisheye_params_private*) obj;

	return 0;
}

static int get_param(void *obj, const char *param, char *value_str, int size) {
	fisheye_params_private *_this = (fisheye_params_private*) obj;
	if (_this->super.next_streamer) {
		_this->super.next_streamer->get_param(_this->super.next_streamer, param,
				value_str, size);
	}

	return 0;
}

static void create_vstreamer(void *user_data, VSTREAMER_T **output_encoder) {
	VSTREAMER_T *encoder = (VSTREAMER_T*) malloc(
			sizeof(fisheye_params_private));
	memset(encoder, 0, sizeof(fisheye_params_private));
	strcpy(encoder->name, VSTREAMER_NAME);
	encoder->release = release;
	encoder->start = start;
	encoder->stop = stop;
	encoder->set_param = set_param;
	encoder->get_param = get_param;
	encoder->get_image = get_image;
	encoder->user_data = encoder;

	fisheye_params_private *_private = (fisheye_params_private*) encoder;

	if (output_encoder) {
		*output_encoder = encoder;
	}
}

static int _command_handler(int argc, char *argv[]) {
	char *cmd = argv[0];
	if (cmd == NULL) {
		//do nothing
	} else if (strcmp(cmd, "set_cam_c") == 0) {
		if (argv[1] == NULL) {
			return -1;
		}
		int cam_num = 0;
		float values[2] = { };
		sscanf(argv[1], "%d=%f,%f", &cam_num, &values[0], &values[1]);
		if (cam_num >= 0 && cam_num < MAX_CAM_NUM) {
			for (int i = 0; i < 2; i++) {
				lg_plugin->fisheye_params[cam_num].cam_c[i] = values[i];
			}
		}

		printf("set_cam_c : completed\n");
	} else if (strcmp(cmd, "set_cam_f") == 0) {
		if (argv[1] == NULL) {
			return -1;
		}
		int cam_num = 0;
		float values[2] = { };
		sscanf(argv[1], "%d=%f,%f", &cam_num, &values[0], &values[1]);
		if (cam_num >= 0 && cam_num < MAX_CAM_NUM) {
			for (int i = 0; i < 2; i++) {
				lg_plugin->fisheye_params[cam_num].cam_f[i] = values[i];
			}
		}

		printf("set_cam_f : completed\n");
	} else if (strcmp(cmd, "set_lens_k") == 0) {
		if (argv[1] == NULL) {
			return -1;
		}
		int cam_num = 0;
		float values[4] = { };
		sscanf(argv[1], "%d=%f,%f,%f,%f", &cam_num, &values[0], &values[1],
				&values[2], &values[3]);
		if (cam_num >= 0 && cam_num < MAX_CAM_NUM) {
			for (int i = 0; i < 4; i++) {
				lg_plugin->fisheye_params[cam_num].lens_k[i] = values[i];
			}
		}

		printf("set_lens_k : completed\n");
	}
	return 0;
}
static int command_handler(void *user_data, const char *buff) {
	int ret = 0;

	wordexp_t p;
	ret = wordexp(buff, &p, 0);
	if (ret != 0) {
		printf("command parse error : %s", buff);
	} else {
		ret = _command_handler(p.we_wordc, p.we_wordv);
	}
	wordfree(&p);

	return ret;
}

static void event_handler(void *user_data, uint32_t node_id,
		uint32_t event_id) {
}

static void init_options(void *user_data, json_t *_options) {
	PLUGIN_T *plugin = (PLUGIN_T*) user_data;
	json_t *options = json_object_get(_options, PLUGIN_NAME);
	if (options == NULL) {
		return;
	}
	{
		json_t *ary = json_object_get(options, "cam_c");
		if (json_is_array(ary)) {
			int size = json_array_size(ary);
			for (int i = 0; i < size && i < MAX_CAM_NUM; i++) {
				json_t *ary2 = json_array_get(ary, i);
				int size2 = MIN(json_array_size(ary2), 4);
				for (int j = 0; j < size2; j++) {
					lg_plugin->fisheye_params[i].cam_c[j] = json_number_value(
							json_array_get(ary2, j));
				}
			}
		}
		for (int i = 0; i < MAX_CAM_NUM; i++) {
			if (lg_plugin->fisheye_params[i].cam_c[0] == 0) {
				for (int j = 0; j < 2; j++) {
					lg_plugin->fisheye_params[i].cam_c[j] = 0.5;
				}
			}
		}
	}
	{
		json_t *ary = json_object_get(options, "cam_f");
		if (json_is_array(ary)) {
			int size = json_array_size(ary);
			for (int i = 0; i < size && i < MAX_CAM_NUM; i++) {
				json_t *ary2 = json_array_get(ary, i);
				int size2 = MIN(json_array_size(ary2), 2);
				for (int j = 0; j < size2; j++) {
					lg_plugin->fisheye_params[i].cam_f[j] = json_number_value(
							json_array_get(ary2, j));
				}
			}
		}
		for (int i = 0; i < MAX_CAM_NUM; i++) {
			if (lg_plugin->fisheye_params[i].cam_f[0] == 0) {
				lg_plugin->fisheye_params[i].cam_f[0] = 0.5 / 1.333;
				lg_plugin->fisheye_params[i].cam_f[1] = 0.5;
			}
		}
	}
	{
		json_t *ary = json_object_get(options, "lens_k");
		if (json_is_array(ary)) {
			int size = json_array_size(ary);
			for (int i = 0; i < size && i < MAX_CAM_NUM; i++) {
				json_t *ary2 = json_array_get(ary, i);
				int size2 = MIN(json_array_size(ary2), 4);
				for (int j = 0; j < size2; j++) {
					lg_plugin->fisheye_params[i].lens_k[j] = json_number_value(
							json_array_get(ary2, j));
				}
			}
		}
		for (int i = 0; i < MAX_CAM_NUM; i++) {
			if (lg_plugin->fisheye_params[i].lens_k[0] == 0) {
				lg_plugin->fisheye_params[i].lens_k[0] = 1.0 / (-3 * 2 * 1);
				lg_plugin->fisheye_params[i].lens_k[1] = 1.0
						/ (5 * 4 * 3 * 2 * 1);
				lg_plugin->fisheye_params[i].lens_k[2] = 1.0
						/ (-7 * 6 * 5 * 4 * 3 * 2 * 1);
				lg_plugin->fisheye_params[i].lens_k[3] = 1.0
						/ (9 * 8 * 7 * 6 * 5 * 4 * 3 * 2 * 1);
			}
		}
	}
	{
		json_t *ary = json_object_get(options, "cam_offset");
		if (json_is_array(ary)) {
			int size = json_array_size(ary);
			for (int i = 0; i < size && i < MAX_CAM_NUM; i++) {
				json_t *ary2 = json_array_get(ary, i);
				int size2 = MIN(json_array_size(ary2), 3);
				for (int j = 0; j < size2; j++) {
					lg_plugin->fisheye_params[i].cam_offset[j] =
							json_number_value(json_array_get(ary2, j));
				}
			}
		}
	}
}

static void save_options(void *user_data, json_t *_options) {
	json_t *options = json_object();
	json_object_set_new(_options, PLUGIN_NAME, options);
	{
		json_t *ary = json_array();
		if (json_is_array(ary)) {
			int size = MAX_CAM_NUM;
			for (int i = 0; i < size; i++) {
				json_t *ary2 = json_array();
				int size2 = 2;
				for (int j = 0; j < size2; j++) {
					json_array_append_new(ary2,
							json_real(lg_plugin->fisheye_params[i].cam_c[j]));
				}
				json_array_append_new(ary, ary2);
			}
		}
		json_object_set_new(options, "cam_c", ary);
	}
	{
		json_t *ary = json_array();
		if (json_is_array(ary)) {
			int size = MAX_CAM_NUM;
			for (int i = 0; i < size; i++) {
				json_t *ary2 = json_array();
				int size2 = 2;
				for (int j = 0; j < size2; j++) {
					json_array_append_new(ary2,
							json_real(lg_plugin->fisheye_params[i].cam_f[j]));
				}
				json_array_append_new(ary, ary2);
			}
		}
		json_object_set_new(options, "cam_f", ary);
	}
	{
		json_t *ary = json_array();
		if (json_is_array(ary)) {
			int size = MAX_CAM_NUM;
			for (int i = 0; i < size; i++) {
				json_t *ary2 = json_array();
				int size2 = 4;
				for (int j = 0; j < size2; j++) {
					json_array_append_new(ary2,
							json_real(lg_plugin->fisheye_params[i].lens_k[j]));
				}
				json_array_append_new(ary, ary2);
			}
		}
		json_object_set_new(options, "lens_k", ary);
	}
	{
		json_t *ary = json_array();
		if (json_is_array(ary)) {
			int size = MAX_CAM_NUM;
			for (int i = 0; i < size; i++) {
				json_t *ary2 = json_array();
				int size2 = 3;
				for (int j = 0; j < size2; j++) {
					json_array_append_new(ary2,
							json_real(
									lg_plugin->fisheye_params[i].cam_offset[j]));
				}
				json_array_append_new(ary, ary2);
			}
		}
		json_object_set_new(options, "cam_offset", ary);
	}
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	lg_plugin_host = plugin_host;

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(plugin_private));
		memset(plugin, 0, sizeof(plugin_private));
		strcpy(plugin->name, PLUGIN_NAME);
		plugin->release = release;
		plugin->command_handler = command_handler;
		plugin->event_handler = event_handler;
		plugin->init_options = init_options;
		plugin->save_options = save_options;
		plugin->get_info = NULL;
		plugin->user_data = plugin;

		*_plugin = plugin;

		lg_plugin = (plugin_private*) plugin;
	}
	{
		VSTREAMER_FACTORY_T *factory = (VSTREAMER_FACTORY_T*) malloc(
				sizeof(VSTREAMER_FACTORY_T));
		memset(factory, 0, sizeof(VSTREAMER_FACTORY_T));
		strcpy(factory->name, VSTREAMER_NAME);
		factory->release = release;
		factory->create_vstreamer = create_vstreamer;
		factory->user_data = factory;

		lg_plugin_host->add_vstreamer_factory(factory);
	}
}
