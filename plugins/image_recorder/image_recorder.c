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
#include <dlfcn.h>
#include <assert.h>

#include "image_recorder.h"
#include "mrevent.h"

#define PLUGIN_NAME "image_recorder"
#define STREAMER_NAME "image_recorder"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static PLUGIN_HOST_T *lg_plugin_host = NULL;

enum RECORDER_MODE {
	RECORDER_MODE_IDLE, RECORDER_MODE_RECORD, RECORDER_MODE_PLAY
};

typedef struct _image_recorder_private {
	VSTREAMER_T super;

	char base_path[257];
	enum RECORDER_MODE mode;
	int framecount;
	int framecount_offset;
	int fps;
	struct timeval last_timestamp;
	struct timeval base_timestamp;
} image_recorder_private;

static void start(void *user_data) {
	image_recorder_private *_this = (image_recorder_private*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}
}

static void stop(void *user_data) {
	image_recorder_private *_this = (image_recorder_private*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static int get_image(void *obj, PICAM360_IMAGE_T **image_p, int *num_p,
		int wait_usec) {
	image_recorder_private *_this = (image_recorder_private*) obj;

	switch (_this->mode) {
	case RECORDER_MODE_RECORD: {
		if (_this->super.pre_streamer == NULL) {
			usleep(wait_usec);
			break;
		}
		int ret;
		int num = MAX_CAM_NUM;
		PICAM360_IMAGE_T *images[MAX_CAM_NUM];
		ret = _this->super.pre_streamer->get_image(_this->super.pre_streamer,
				images, &num, wait_usec);
		if (ret != 0) {
			break;
		}
		char path[512];
		snprintf(path, sizeof(path) - 1, "%s/%d.pif", _this->base_path,
				_this->framecount);

		ret = save_picam360_image_from_file(path, images, num);
		if (ret == 0) {
			_this->framecount++;
			return 0;
		}
		break;
	}
	case RECORDER_MODE_PLAY: {
		char path[512];
		sprintf(path, "%s/%d.pif", _this->base_path,
				_this->framecount - _this->framecount_offset);

		int ret = load_picam360_image_from_file(path, image_p, num_p);
		if (ret == 0) {
			struct timeval diff;
			float elapsed_sec;

			struct timeval now;
			gettimeofday(&now, NULL);
			if (_this->fps <= 0) {
				if (_this->framecount >= 1) {
					timersub(&image_p[0]->timestamp, &_this->last_timestamp, &diff);
					elapsed_sec = (float) diff.tv_sec
							+ (float) diff.tv_usec / 1000000;
					_this->fps = (int) (1.0 / elapsed_sec + 0.5);
				}
			} else {
				float elapsed_sec_target = (float) _this->framecount
						/ _this->fps;

				timersub(&now, &_this->base_timestamp, &diff);
				elapsed_sec = (float) diff.tv_sec
						+ (float) diff.tv_usec / 1000000;
				elapsed_sec_target -= elapsed_sec;
				if (elapsed_sec_target > 10) {
					elapsed_sec_target = 10;
					printf("something wrong : %s\n", __FILE__);
				}
				if (elapsed_sec_target > 0) {
					usleep(elapsed_sec_target * 1000000);
				}
			}
			_this->last_timestamp = image_p[0]->timestamp;
			gettimeofday(&image_p[0]->timestamp, NULL);
			_this->framecount++;
			return 0;
		}
		_this->framecount_offset = _this->framecount;
		usleep(wait_usec);
		break;
	}
	case RECORDER_MODE_IDLE:
	default:
		if (_this->super.pre_streamer == NULL) {
			usleep(wait_usec);
			break;
		}
		return _this->super.pre_streamer->get_image(_this->super.pre_streamer,
				image_p, num_p, wait_usec);
	}
	return -1;
}

static int set_param(void *obj, const char *param, const char *value_str) {
	image_recorder_private *_this = (image_recorder_private*) obj;
	if (strcmp(param, "mode") == 0) {
		_this->mode = RECORDER_MODE_IDLE;
		_this->framecount = 0;
		_this->framecount_offset = 0;
		gettimeofday(&_this->base_timestamp, NULL);
		if (strcmp(value_str, "IDLE") == 0) {
			//do nothing
		} else if (strcmp(value_str, "RECORD") == 0) {
			_this->mode = RECORDER_MODE_RECORD;
		} else if (strcmp(value_str, "PLAY") == 0) {
			_this->mode = RECORDER_MODE_PLAY;
		}
	} else if (strcmp(param, "base_path") == 0) {
		int len = snprintf(_this->base_path, sizeof(_this->base_path) - 1, "%s",
				value_str);
		if (_this->base_path[len - 1] == '/') {
			_this->base_path[len - 1] = '\0';
			len--;
		}
	}
}

static int get_param(void *obj, const char *param, char *value, int size) {
}

static void release(void *obj) {
	image_recorder_private *_this = (image_recorder_private*) obj;

	free(obj);
}

static void create_vstreamer(void *user_data, VSTREAMER_T **output_streamer) {
	VSTREAMER_T *streamer = (VSTREAMER_T*) malloc(
			sizeof(image_recorder_private));
	memset(streamer, 0, sizeof(image_recorder_private));
	strcpy(streamer->name, STREAMER_NAME);
	streamer->release = release;
	streamer->start = start;
	streamer->stop = stop;
	streamer->set_param = set_param;
	streamer->get_param = get_param;
	streamer->get_image = get_image;
	streamer->user_data = streamer;

	image_recorder_private *_private = (image_recorder_private*) streamer;

	if (output_streamer) {
		*output_streamer = streamer;
	}
}

static int command_handler(void *user_data, const char *_buff) {
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
}

static void init_options(void *user_data, json_t *_options) {
	PLUGIN_T *plugin = (PLUGIN_T*) user_data;
	json_t *options = json_object_get(_options, PLUGIN_NAME);
	if (options == NULL) {
		return;
	}
}

static void save_options(void *user_data, json_t *_options) {
	json_t *options = json_object();
	json_object_set_new(_options, PLUGIN_NAME, options);
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	lg_plugin_host = plugin_host;

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(PLUGIN_T));
		memset(plugin, 0, sizeof(PLUGIN_T));
		strcpy(plugin->name, PLUGIN_NAME);
		plugin->release = release;
		plugin->command_handler = command_handler;
		plugin->event_handler = event_handler;
		plugin->init_options = init_options;
		plugin->save_options = save_options;
		plugin->get_info = NULL;
		plugin->user_data = plugin;

		*_plugin = plugin;
	}
	{
		VSTREAMER_FACTORY_T *factory = (VSTREAMER_FACTORY_T*) malloc(
				sizeof(VSTREAMER_FACTORY_T));
		memset(factory, 0, sizeof(VSTREAMER_FACTORY_T));
		strcpy(factory->name, STREAMER_NAME);
		factory->release = release;
		factory->create_vstreamer = create_vstreamer;
		factory->user_data = factory;

		lg_plugin_host->add_vstreamer_factory(factory);
	}
}
