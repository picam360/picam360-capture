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

#include "rtp_streamer.h"
#include "mrevent.h"

#define PLUGIN_NAME "rtp_streamer"
#define STREAMER_NAME "rtp"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static PLUGIN_HOST_T *lg_plugin_host = NULL;

#define BUFFER_NUM 4

typedef struct _rtp_streamer_private {
	VSTREAMER_T super;

	int port;

	bool run;
	pthread_t streaming_thread;
} rtp_streamer_private;

static size_t _write(void *user_data, void *data, size_t data_len, int img_num,
		int plane_num, char *img_type) {
	RTP_T *rtp = (RTP_T*) user_data;
	for (int j = 0; j < data_len;) {
		int len;
		if (j + RTP_MAXPAYLOADSIZE < data_len) {
			len = RTP_MAXPAYLOADSIZE;
		} else {
			len = data_len - j;
		}
		rtp_sendpacket(rtp, data + j, len, PT_CAM_BASE);
		rtp_flush(rtp);
		j += len;
	}
	return data_len;
}

static void send_image(RTP_T *rtp, PICAM360_IMAGE_T **image, int num) {
	if(num > 1)
	{
		unsigned char soia[4] = { 'S', 'O', 'I', 'A' }; //start of image array
		rtp_sendpacket(rtp, soia, 4, PT_CAM_BASE);
		rtp_flush(rtp);
	}

	save_picam360_image(images, num, _write, (void*) rtp);

	if(num > 1)
	{
		unsigned char eoia[4] = { 'E', 'O', 'I', 'A' }; //end of image array
		rtp_sendpacket(rtp, eoia, 4, PT_CAM_BASE);
		rtp_flush(rtp);
	}
}

static void* streaming_thread_fnc(void *obj) {
	rtp_streamer_private *_this = (rtp_streamer_private*) obj;

	RTP_T *rtp = NULL;
	if (_this->port != 0) {
		rtp = create_rtp(0, 0, "127.0.0.1", _this->port, RTP_SOCKET_TYPE_UDP,
				0);
	} else {
		rtp = lg_plugin_host->get_rtp();
	}

	while (_this->run) {
		if (_this->super.pre_streamer == NULL) {
			usleep(100 * 1000);
			continue;
		}
		int ret;
		int num = MAX_CAM_NUM;
		PICAM360_IMAGE_T *images[MAX_CAM_NUM];
		ret = _this->super.pre_streamer->get_image(_this->super.pre_streamer,
				images, &num, 100 * 1000);
		if (ret != 0) {
			continue;
		}
		if (image->mem_type != PICAM360_MEMORY_TYPE_PROCESS) {
			printf("%s : something wrong!\n", __FILE__);
			continue;
		}

		send_image(rtp, images, num);

		if (image->ref) {
			image->ref->release(image->ref);
		}

		{
			static int count = 0;
			static float elapsed_sec = 0;
			float _elapsed_sec;
			struct timeval diff;

			struct timeval now;
			gettimeofday(&now, NULL);

			timersub(&now, &image->timestamp, &diff);
			_elapsed_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
			if ((count % 200) == 0) {
				printf("rtp : %f\n", _elapsed_sec);
			}
			count++;
		}
	}

	if (_this->port != 0) {
		delete_rtp(&rtp);
	}

	return NULL;
}

static void start(void *user_data) {
	rtp_streamer_private *_this = (rtp_streamer_private*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}

	_this->run = true;
	pthread_create(&_this->streaming_thread, NULL, streaming_thread_fnc,
			user_data);
}

static void stop(void *user_data) {
	rtp_streamer_private *_this = (rtp_streamer_private*) user_data;

	if (_this->run) {
		_this->run = false;
		pthread_join(_this->streaming_thread, NULL);
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static int get_image(void *obj, PICAM360_IMAGE_T **image_p, int *num_p,
		int wait_usec) {
	rtp_streamer_private *_this = (rtp_streamer_private*) obj;

	return -1;
}

static int set_param(void *obj, const char *param, const char *value_str) {
	rtp_streamer_private *_this = (rtp_streamer_private*) obj;
	if (strcmp(param, "port") == 0) {
		int value = 0;
		sscanf(value_str, "%d", &value);
		if (9100 <= value && value < 10000) {
			_this->port = value;
		} else {
			printf("error: port=%d is out of range\n", value);
		}
	}
}

static int get_param(void *obj, const char *param, char *value, int size) {
}

static void release(void *obj) {
	rtp_streamer_private *_this = (rtp_streamer_private*) obj;

	if (_this->run) {
		_this->super.stop(&_this->super);
	}

	free(obj);
}

static void create_vstreamer(void *user_data, VSTREAMER_T **output_streamer) {
	VSTREAMER_T *streamer = (VSTREAMER_T*) malloc(sizeof(rtp_streamer_private));
	memset(streamer, 0, sizeof(rtp_streamer_private));
	strcpy(streamer->name, STREAMER_NAME);
	streamer->release = release;
	streamer->start = start;
	streamer->stop = stop;
	streamer->set_param = set_param;
	streamer->get_param = get_param;
	streamer->get_image = get_image;
	streamer->user_data = streamer;

	rtp_streamer_private *_private = (rtp_streamer_private*) streamer;

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
