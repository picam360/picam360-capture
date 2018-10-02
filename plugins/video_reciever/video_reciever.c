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

#include "video_reciever.h"

#define PT_CAM_BASE 110

#define PLUGIN_NAME "video_reciever"
#define CAPTURE_NAME "video_reciever"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

typedef struct _video_reciever {
	CAPTURE_T super;

	int cam_num;

	void *user_data;
} video_reciever;

static void release(void *obj) {
	free(obj);
}
static int rtp_callback(unsigned char *data, unsigned int data_len, unsigned char pt, unsigned int seq_num, void *user_data) {
	video_reciever *_this = (video_reciever*) user_data;

	if (data_len == 0) {
		return -1;
	}

	if (pt == PT_CAM_BASE + _this->cam_num) {
		lg_plugin_host->decode_video(_this->cam_num, (unsigned char*) data, data_len);
	}
	return 0;
}
static void start(void *obj, int cam_num, void *display, void *context, int egl_image_num) {
	video_reciever *_this = (video_reciever*) obj;

	_this->cam_num = cam_num;

	rtp_add_callback(lg_plugin_host->get_rtp(), (RTP_CALLBACK) rtp_callback, (void*) _this);
}

static float get_fps(void *user_data) {
	return 0;
}

static void create_capture(void *user_data, CAPTURE_T **out_capture) {
	CAPTURE_T *capture = (CAPTURE_T*) malloc(sizeof(video_reciever));
	memset(capture, 0, sizeof(video_reciever));
	strcpy(capture->name, CAPTURE_NAME);
	capture->release = release;
	capture->start = start;
	capture->get_fps = get_fps;
	capture->user_data = capture;

	if (out_capture) {
		*out_capture = capture;
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
		CAPTURE_FACTORY_T *capture_factory = (CAPTURE_FACTORY_T*) malloc(sizeof(CAPTURE_FACTORY_T));
		memset(capture_factory, 0, sizeof(CAPTURE_FACTORY_T));
		strcpy(capture_factory->name, CAPTURE_NAME);
		capture_factory->release = release;
		capture_factory->create_capture = create_capture;

		lg_plugin_host->add_capture_factory(capture_factory);
	}
}
