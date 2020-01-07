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

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <thread>
#include "omxcv.h"
//#include <opencv2/opencv.hpp>

#if __linux
#include <sys/prctl.h>
#endif

#include "omx_encoder.h"

#define PLUGIN_NAME "omx_encoder"
#define MJPEG_ENCODER_NAME "mjpeg"
#define H264_ENCODER_NAME "h264"

#define TIMEDIFF(start) (duration_cast<microseconds>(steady_clock::now() - start).count())

using omxcv::OmxCv;
using omxcv::OmxCvJpeg;
using std::this_thread::sleep_for;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

static PLUGIN_HOST_T *lg_plugin_host = NULL;

typedef struct _omx_encoder {
	ENCODER_T super;

	const char *codec;
	OmxCv *omxcv;
} omx_encoder;

static void init(void *obj, const int width, const int height, int bitrate_kbps, int fps, ENCODER_STREAM_CALLBACK callback, void *user_data) {
	omx_encoder *_this = (omx_encoder*) obj;

	_this->omxcv = new OmxCv(_this->codec, width, height, bitrate_kbps, fps, 1, callback, user_data);
}
static void release(void *obj) {
	omx_encoder *_this = (omx_encoder*) obj;
	if (_this->omxcv) {
		delete _this->omxcv;
	}
	free(obj);
}
static void add_frame(void *obj, const unsigned char *in_data, void *frame_data) {
	omx_encoder *_this = (omx_encoder*) obj;

	if (_this->omxcv) {
		_this->omxcv->Encode(in_data, frame_data);
	}
}

static void create_mjpeg_encoder(void *obj, ENCODER_T **out_encoder) {
	ENCODER_T *encoder = (ENCODER_T*) malloc(sizeof(omx_encoder));
	memset(encoder, 0, sizeof(omx_encoder));
	strncpy(encoder->name, MJPEG_ENCODER_NAME, sizeof(encoder->name));
	encoder->release = release;
	encoder->init = init;
	encoder->add_frame = add_frame;
	encoder->user_data = encoder;
	((omx_encoder*)encoder)->codec = "dummy.mjpeg";

	if (out_encoder) {
		*out_encoder = encoder;
	}
}

static void create_h264_encoder(void *obj, ENCODER_T **out_encoder) {
	ENCODER_T *encoder = (ENCODER_T*) malloc(sizeof(omx_encoder));
	memset(encoder, 0, sizeof(omx_encoder));
	strncpy(encoder->name, H264_ENCODER_NAME, sizeof(encoder->name));
	encoder->release = release;
	encoder->init = init;
	encoder->add_frame = add_frame;
	encoder->user_data = encoder;
	((omx_encoder*)encoder)->codec = "dummy.h264";

	if (out_encoder) {
		*out_encoder = encoder;
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
	{//mjpeg
		ENCODER_FACTORY_T *encoder_factory = (ENCODER_FACTORY_T*) malloc(sizeof(ENCODER_FACTORY_T));
		memset(encoder_factory, 0, sizeof(ENCODER_FACTORY_T));
		strcpy(encoder_factory->name, MJPEG_ENCODER_NAME);
		encoder_factory->release = release;
		encoder_factory->create_encoder = create_mjpeg_encoder;

		lg_plugin_host->add_encoder_factory(encoder_factory);
	}
	{//h264
		ENCODER_FACTORY_T *encoder_factory = (ENCODER_FACTORY_T*) malloc(sizeof(ENCODER_FACTORY_T));
		memset(encoder_factory, 0, sizeof(ENCODER_FACTORY_T));
		strcpy(encoder_factory->name, H264_ENCODER_NAME);
		encoder_factory->release = release;
		encoder_factory->create_encoder = create_h264_encoder;

		lg_plugin_host->add_encoder_factory(encoder_factory);
	}
}
