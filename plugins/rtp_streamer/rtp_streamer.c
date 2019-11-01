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

typedef struct _RTP_LIST_T {
	int port;
	RTP_T *rtp;

	struct _RTP_LIST_T *next;
} RTP_LIST_T;
static RTP_LIST_T *lg_rpt_list = NULL;

typedef struct _rtp_streamer_private {
	VSTREAMER_T super;

	int port;

	bool run;
	pthread_t streaming_thread;
} rtp_streamer_private;

typedef struct _NALU_STREAM_DEF {
	unsigned char SC[4];
	unsigned char SOI[2];
	unsigned char EOI[2];
	unsigned char SEI_CODE;
} NALU_STREAM_DEF;

static void send_xmp(RTP_T *rtp, PICAM360_IMAGE_T *image, NALU_STREAM_DEF def) {
	unsigned char header_pack[512];
	char *xmp = (char*) header_pack + sizeof(def.SOI);
	int len = image->meta_size;
	memcpy(xmp + 4, image->meta, image->meta_size);
	xmp[2] = (len >> 8) & 0xFF;
	xmp[3] = (len >> 0) & 0xFF;
	len += 2; //length code
	xmp[0] = 0xFF;
	xmp[1] = 0xE1;
	len += 2; //xmp code
	memcpy(header_pack, def.SOI, sizeof(def.SOI));
	len += 2; //SOI
	rtp_sendpacket(rtp, header_pack, len, PT_CAM_BASE);
}

static void send_sei(RTP_T *rtp, PICAM360_IMAGE_T *image, NALU_STREAM_DEF def) {
	unsigned char header_pack[512];
	char *sei = (char*) header_pack + sizeof(def.SOI);
	int len = image->meta_size;
	sei[4] = def.SEI_CODE; //nal_type:sei
	len += 1; //nal header
	memcpy(sei + 5, image->meta, image->meta_size);
	sei[0] = (len >> 24) & 0xFF;
	sei[1] = (len >> 16) & 0xFF;
	sei[2] = (len >> 8) & 0xFF;
	sei[3] = (len >> 0) & 0xFF;
	len += 4; //start code
	memcpy(header_pack, def.SOI, sizeof(def.SOI));
	len += 2; //SOI
	rtp_sendpacket(rtp, header_pack, len, PT_CAM_BASE);
}

static void _send_image(RTP_T *rtp, unsigned char *data, unsigned int data_len) {
	for (int j = 0; j < data_len;) {
		int len;
		if (j + RTP_MAXPAYLOADSIZE < data_len) {
			len = RTP_MAXPAYLOADSIZE;
		} else {
			len = data_len - j;
		}
		rtp_sendpacket(rtp, data + j, len, PT_CAM_BASE);
		j += len;
	}
}

static void send_image(RTP_T *rtp, PICAM360_IMAGE_T *image) {
	unsigned char *data = image->pixels[0];
	unsigned int data_len = image->stride[0];
	NALU_STREAM_DEF def = { };
	if (memcmp(image->img_type, "I420", 4) == 0) {
		NALU_STREAM_DEF _def = { .SC = { 0x00, 0x00, 0x00, 0x01 }, .SOI = {
				0x49, 0x34 }, //'I', '4'
				.EOI = { 0x32, 0x30 }, //'2', '0'
				.SEI_CODE = 40 << 1, };
		def = _def;
	} else if (memcmp(image->img_type, "H265", 4) == 0) {
		NALU_STREAM_DEF _def = { .SC = { 0x00, 0x00, 0x00, 0x01 }, .SOI = {
				0x48, 0x45 }, //'H', 'E'
				.EOI = { 0x56, 0x43 }, //'V', 'C'
				.SEI_CODE = 40 << 1, };
		def = _def;
	} else if (memcmp(image->img_type, "H264", 4) == 0) {
		NALU_STREAM_DEF _def = { .SC = { 0x00, 0x00, 0x00, 0x01 }, .SOI = {
				0x4E, 0x41 }, //'N', 'A'
				.EOI = { 0x4C, 0x55 }, //'L', 'U'
				.SEI_CODE = 6, };
		def = _def;
	} else if (memcmp(image->img_type, "JPEG", 4) == 0) {
		NALU_STREAM_DEF _def = { .SC = { 0x00, 0x00, 0x00, 0x01 }, .SOI = {
				0xFF, 0xD8 }, .EOI = { 0xFF, 0xD9 }, .SEI_CODE = 0, };
		def = _def;
	}

	if (memcmp(image->img_type, "I420", 4) == 0
			|| memcmp(image->img_type, "H265", 4) == 0
			|| memcmp(image->img_type, "H264", 4) == 0) {
		//header pack
		send_sei(rtp, image, def);
		rtp_flush(rtp);
		_send_image(rtp, data, data_len);
		rtp_sendpacket(rtp, def.EOI, sizeof(def.EOI), PT_CAM_BASE);
		rtp_flush(rtp);
	} else if (memcmp(image->img_type, "JPEG", 4) == 0) {
		//header pack
		send_xmp(rtp, image, def);
		rtp_flush(rtp);
		{
			int cur = 0;
			for (; cur < data_len; cur++) {
				if (data[cur + 0] == def.SOI[0]
						&& data[cur + 1] == def.SOI[1]) {
					break;
				}
			}
			for (; cur < data_len; data_len--) {
				if (data[data_len - 2] == def.EOI[0]
						&& data[data_len - 1] == def.EOI[1]) {
					break;
				}
			}
			_send_image(rtp, data + (cur + 2), (data_len - 2) - (cur + 2));
		}
		rtp_sendpacket(rtp, def.EOI, sizeof(def.EOI), PT_CAM_BASE);
		rtp_flush(rtp);
	}
}

static void* streaming_thread_fnc(void *obj) {
	rtp_streamer_private *_this = (rtp_streamer_private*) obj;

	RTP_T *rtp = NULL;
	if (_this->port == 0) {
		rtp = lg_plugin_host->get_rtp();
	} else {
		RTP_LIST_T **tail_p = &lg_rpt_list;
		for (; (*tail_p) != NULL; tail_p = &(*tail_p)->next) {
			if ((*tail_p)->port == _this->port) {
				break;
			}
		}
		if ((*tail_p) == NULL) {
			RTP_LIST_T *list = (RTP_LIST_T*) malloc(sizeof(RTP_LIST_T));
			list->port = _this->port;
			list->rtp = create_rtp(0, 0, "127.0.0.1", _this->port,
					RTP_SOCKET_TYPE_UDP, 0);
			*tail_p = list;
		}
		rtp = (*tail_p)->rtp;
	}

	while (_this->run) {
		if (_this->super.pre_streamer == NULL) {
			usleep(100 * 1000);
			continue;
		}
		int ret;
		int num;
		PICAM360_IMAGE_T *image;
		ret = _this->super.pre_streamer->get_image(_this->super.pre_streamer,
				&image, &num, 100 * 1000);
		if (ret != 0) {
			continue;
		}
		if (num != 1 || image->mem_type != PICAM360_MEMORY_TYPE_PROCESS) {
			printf("%s : something wrong!\n", __FILE__);
			continue;
		}

		send_image(rtp, image);

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
		_this->port = MAX(9100, value);
	}
}

static int get_param(void *obj, const char *param, const char *value, int size) {
}

static void release(void *obj) {
	rtp_streamer_private *_this = (rtp_streamer_private*) obj;

	free(obj);
}

static void create_vstreamer(void *user_data,
		VSTREAMER_T **output_streamer) {
	VSTREAMER_T *streamer = (VSTREAMER_T*) malloc(sizeof(rtp_streamer_private));
	memset(streamer, 0, sizeof(rtp_streamer_private));
	strcpy(streamer->name, STREAMER_NAME);
	streamer->release = release;
	streamer->start = start;
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
