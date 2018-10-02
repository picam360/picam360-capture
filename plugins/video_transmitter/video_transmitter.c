#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#if __linux
#include <sys/prctl.h>
#endif

#include "video_transmitter.h"

#define PT_CAM_BASE 110

#define PLUGIN_NAME "video_transmitter"
#define DECODER_NAME "video_transmitter"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

typedef struct _video_transmitter {
	DECODER_T super;

	int cam_num;

	void *user_data;
} video_transmitter;

static void init(void *obj, int cam_num, void *display, void *context, void *cam_texture, void **egl_images, int egl_image_num) {
	video_transmitter *_this = (video_transmitter*) obj;

	_this->cam_num = cam_num;
}

static void release(void *obj) {
	video_transmitter *_this = (video_transmitter*) obj;
	free(obj);
}

static void decode(void *user_data, unsigned char *data, int data_len) {
	video_transmitter *_this = (video_transmitter*) user_data;

	for (int i = 0; i < data_len;) {
		int len;
		if ((data_len - i) > RTP_MAXPAYLOADSIZE) {
			len = RTP_MAXPAYLOADSIZE;
		} else {
			len = (data_len - i);
		}
		rtp_sendpacket(lg_plugin_host->get_rtp(), data + i, len, PT_CAM_BASE + _this->cam_num);
		i += len;
	}
	if(data[data_len - 1] == 0xD9 && data[data_len - 2] == 0xFF){
		rtp_flush(lg_plugin_host->get_rtp());
	}
}

static float get_fps(void *user_data) {
	return 0;
}

static int get_frameskip(void *user_data) {
	return 0;
}

static void switch_buffer(void *user_data) {
}

static void create_decoder(void *user_data, DECODER_T **out_decoder) {
	DECODER_T *decoder = (DECODER_T*) malloc(sizeof(video_transmitter));
	memset(decoder, 0, sizeof(video_transmitter));
	strcpy(decoder->name, DECODER_NAME);
	decoder->release = release;
	decoder->init = init;
	decoder->get_fps = get_fps;
	decoder->get_frameskip = get_frameskip;
	decoder->decode = decode;
	decoder->switch_buffer = switch_buffer;
	decoder->user_data = decoder;

	if (out_decoder) {
		*out_decoder = decoder;
	}
}

static int command_handler(void *user_data, const char *_buff) {
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
}

static void init_options(void *user_data, json_t *options) {
}

static void save_options(void *user_data, json_t *options) {
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
		DECODER_FACTORY_T *decoder_factory = (DECODER_FACTORY_T*) malloc(sizeof(DECODER_FACTORY_T));
		memset(decoder_factory, 0, sizeof(DECODER_FACTORY_T));
		strcpy(decoder_factory->name, DECODER_NAME);
		decoder_factory->release = release;
		decoder_factory->create_decoder = create_decoder;

		lg_plugin_host->add_decoder_factory(decoder_factory);
	}
}
