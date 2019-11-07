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
	struct timeval timestamp;
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

static int release_image(void *user_data) {
	PICAM360_IMAGE_T *image = (PICAM360_IMAGE_T*) user_data;
	for (int i = 0; i < 3; i++) {
		if (image->pixels[i] == NULL) {
			continue;
		}
		free(image->pixels[i]);
	}
	if (image->meta != NULL) {
		free(image->meta);
	}
	free(image);
}

static int save_image(char *path, PICAM360_IMAGE_T **images, int num) {
	PICAM360_IMAGE_T *image = images[0];
	int fd = open(path, O_CREAT | O_WRONLY | O_TRUNC,
			S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);

	char uuid_str[37]; //UUID_STR_LEN
	uuid_unparse_upper(image->uuid, uuid_str);

	char buff[4 * 1024];
	int len = 4;
	len += sprintf(buff + len, "<PICAM360_IMAGE_T ");
	len += sprintf(buff + len, "version=\"2.0\" ");
	len += sprintf(buff + len, "uuid=\"%s\" ", uuid_str);
	len += sprintf(buff + len, "timestamp=\"%ld,%ld\" ",
			image->timestamp.tv_sec, image->timestamp.tv_usec);
	len += sprintf(buff + len, "img_type=\"%.4s\" ", image->img_type);
	len += sprintf(buff + len, "meta_size=\"%d\" ", image->meta_size);
	len += sprintf(buff + len, "num_of_planes=\"%d\" ", image->num_of_planes);
	len += sprintf(buff + len, "width=\"%d,%d,%d\" ", image->width[0],
			image->width[1], image->width[2]);
	len += sprintf(buff + len, "stride=\"%d,%d,%d\" ", image->stride[0],
			image->stride[1], image->stride[2]);
	len += sprintf(buff + len, "height=\"%d,%d,%d\" ", image->height[0],
			image->height[1], image->height[2]);
	len += sprintf(buff + len, "/>");
	buff[0] = 'P';
	buff[1] = 'I';
	buff[2] = ((len - 4) & 0xff00) >> 8;
	buff[3] = ((len - 4) & 0x00ff) >> 0;
	int cur = 0;
	cur += write(fd, buff, len);
	cur += write(fd, image->meta, image->meta_size);
	for (int i = 0; i < 3; i++) {
		int size = image->stride[i] * image->height[i];
		if (size == 0) {
			continue;
		}
		cur += write(fd, image->pixels[i], size);
	}
	close(fd);
}

static int load_image(char *path, PICAM360_IMAGE_T **image_p, int *num_p) {
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		return -1;
	}
	PICAM360_IMAGE_T *image = (PICAM360_IMAGE_T*) malloc(
			sizeof(PICAM360_IMAGE_T));
	memset(image, 0, sizeof(PICAM360_IMAGE_T));
	create_reference(&image->ref, release_image, image);

	char buff[4 * 1024];
	int cur = 0;
	cur += read(fd, buff, 4);
	int len = 0;
	len += (int) buff[2] << 8;
	len += (int) buff[3] << 0;
	cur += read(fd, buff + cur, len);
	buff[cur] = '\0';

	const int kMaxArgs = 32;
	int argc = 0;
	char *argv[kMaxArgs];
	char *p = strtok(buff + 4, " \n");
	while (p && argc < kMaxArgs - 1) {
		argv[argc++] = p;
		p = strtok(NULL, " \n");
	}
	for (int p = 0; p < argc; p++) {
		char *name = strtok(argv[p], "=");
		char *value = strtok(NULL, "=");
		if (strcmp(name, "uuid") == 0) {
			char uuid_str[37]; //UUID_STR_LEN
			sscanf(value, "\"%36s\"", uuid_str);
			uuid_parse(uuid_str, image->uuid);
		} else if (strcmp(name, "timestamp") == 0) {
			sscanf(value, "\"%ld,%ld\"", &image->timestamp.tv_sec,
					&image->timestamp.tv_usec);
		} else if (strcmp(name, "img_type") == 0) {
			char img_type[5] = { };
			sscanf(value, "\"%4s\"", img_type);
			memcpy(image->img_type, img_type, 4);
		} else if (strcmp(name, "meta_size") == 0) {
			sscanf(value, "\"%d\"", &image->meta_size);
		} else if (strcmp(name, "num_of_planes") == 0) {
			sscanf(value, "\"%d\"", &image->num_of_planes);
		} else if (strcmp(name, "width") == 0) {
			sscanf(value, "\"%d,%d,%d\"", &image->width[0], &image->width[1],
					&image->width[2]);
		} else if (strcmp(name, "stride") == 0) {
			sscanf(value, "\"%d,%d,%d\"", &image->stride[0], &image->stride[1],
					&image->stride[2]);
		} else if (strcmp(name, "height") == 0) {
			sscanf(value, "\"%d,%d,%d\"", &image->height[0], &image->height[1],
					&image->height[2]);
		}
	}
	if (image->meta_size > 0) {
		image->meta = (unsigned char*) malloc(image->meta_size + 1);
		cur += read(fd, image->meta, image->meta_size);
	}
	for (int i = 0; i < 3; i++) {
		int size = image->stride[i] * image->height[i];
		if (size == 0) {
			continue;
		}
		image->pixels[i] = (unsigned char*) malloc(size);
		cur += read(fd, image->pixels[i], size);
	}
	close(fd);

	*image_p = image;
	*num_p = 1;

	return 0;
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

		ret = save_image(path, images, num);
		if (ret == 0) {
			_this->framecount++;
			return 0;
		}
		break;
	}
	case RECORDER_MODE_PLAY: {
		char path[512];
		sprintf(path, "%s/%d.pif", _this->base_path, _this->framecount);

		int ret = load_image(path, image_p, num_p);
		if (ret == 0) {
			struct timeval diff;
			float elapsed_sec;
			timersub(&image_p[0]->timestamp, &_this->timestamp, &diff);
			elapsed_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
			if (elapsed_sec > 0) {
				usleep(diff.tv_usec);
			}
			_this->timestamp = image_p[0]->timestamp;
			_this->framecount++;
			return 0;
		}
		_this->framecount = 0;
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
		if (strcmp(value_str, "IDLE") == 0) {
			//do nothing
		} else if (strcmp(value_str, "RECORD") == 0) {
			_this->mode = RECORDER_MODE_RECORD;
		} else if (strcmp(value_str, "PLAY") == 0) {
			_this->mode = RECORDER_MODE_PLAY;
		}
	} else if (strcmp(param, "base_path") == 0) {
		int len = snprintf(_this->base_path, sizeof(_this->base_path) - 1, "%s", value_str);
		if(_this->base_path[len - 1] == '/'){
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
