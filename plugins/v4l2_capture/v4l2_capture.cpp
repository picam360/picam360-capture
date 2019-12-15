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
#include <dlfcn.h>
#include <assert.h>
#include <sys/time.h>
#include <limits.h>
#include <list>
#if __linux
#include <sys/prctl.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "mrevent.h"

#ifdef __cplusplus
}
#endif

#include "v4l2_handler.h"
#include "v4l2_capture.h"

#define PLUGIN_NAME "v4l2_capture"
#define CAPTURE_NAME "v4l2_capture"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define BUFFER_NUM 4

typedef struct _v4l2_capture {
	VSTREAMER_T super;

	char vstream_filepath[256];
	bool run;
	int cam_num;
	int cam_width;
	int cam_height;
	int cam_fps;
	int skipframe;
	unsigned int framecount;
	unsigned int recieved_framecount;
	float fps;
	int frameskip;
	PICAM360_IMAGE_T frame_buffers[BUFFER_NUM];
	uint8_t xmp_buffers[BUFFER_NUM][RTP_MAXPAYLOADSIZE];
	MREVENT_T frame_ready;
	pthread_t streaming_thread;
} v4l2_capture;

static PLUGIN_HOST_T *lg_plugin_host = NULL;
static v4l2_capture *lg_v4l2_capture_ary[MAX_CAM_NUM] = { };
static int lg_width = 2048;
static int lg_height = 1536;
static int lg_fps = 15;
static int lg_skipframe = 0;
static int lg_mjpeg_kbps = 0;
static char lg_devicefiles[MAX_CAM_NUM][256] = { };
static pthread_mutex_t lg_frame_mlock = PTHREAD_MUTEX_INITIALIZER;

typedef struct _V4l2_CTL_T {
	char name[256];
	int value;
} V4l2_CTL_T;

V4l2_CTL_T **lg_v4l2_ctls = NULL;

static void set_v4l2_ctl(const char *name, const int value, int cam_num) {
	if (lg_devicefiles[cam_num][0] != '\0') {
		int ret;
		char cmd[512];
		sprintf(cmd, "v4l2-ctl --set-ctrl=%s=%d -d %s", name, value,
				lg_devicefiles[cam_num]);
		printf("shell:%s\n", cmd);
		ret = system(cmd);
	}
}

static void add_v4l2_ctl(V4l2_CTL_T *ctl) {
	if (lg_v4l2_ctls == NULL) {
		const int INITIAL_SPACE = 16;
		lg_v4l2_ctls = (V4l2_CTL_T**) malloc(
				sizeof(V4l2_CTL_T*) * INITIAL_SPACE);
		memset(lg_v4l2_ctls, 0, sizeof(V4l2_CTL_T*) * INITIAL_SPACE);
		lg_v4l2_ctls[INITIAL_SPACE - 1] = (V4l2_CTL_T*) -1;
	}

	for (int i = 0; lg_v4l2_ctls[i] != (V4l2_CTL_T*) -1; i++) {
		if (lg_v4l2_ctls[i] == NULL) {
			lg_v4l2_ctls[i] = ctl;
			return;
		}
		if (lg_v4l2_ctls[i + 1] == (V4l2_CTL_T*) -1) {
			int space = (i + 2) * 2;
			if (space > 256) {
				fprintf(stderr, "error on add_mpu\n");
				return;
			}
			V4l2_CTL_T **current = lg_v4l2_ctls;
			lg_v4l2_ctls = (V4l2_CTL_T**) malloc(sizeof(V4l2_CTL_T*) * space);
			memcpy(lg_v4l2_ctls, current, sizeof(V4l2_CTL_T*) * (i + 1));
			lg_v4l2_ctls[space - 1] = (V4l2_CTL_T*) -1;
			free(current);
		}
	}
}

static int v4l2_progress_image(const void *p, int size,
		struct timeval timestamp, void *arg) {
	v4l2_capture *_this = (v4l2_capture*) arg;

	//remove space
	for (; size > 0;) {
		if (((unsigned char*) p)[size - 1] == 0xD9) {
			break;
		} else {
			size--;
		}
	}
	int cur = _this->recieved_framecount % BUFFER_NUM;

	int xmp_len = lg_plugin_host->xmp((char*) (_this->xmp_buffers[cur]),
			RTP_MAXPAYLOADSIZE, _this->cam_num);

	PICAM360_IMAGE_T *frame = &_this->frame_buffers[cur];
	memset(frame, 0, sizeof(PICAM360_IMAGE_T));
	frame->timestamp = timestamp;
	frame->pixels[0] = (unsigned char*) p;
	frame->meta = _this->xmp_buffers[cur];
	frame->meta_size = xmp_len;
	frame->width[0] = size;
	frame->stride[0] = size;
	frame->height[0] = 1;
	memcpy(frame->img_type, "JPEG", 4);
	frame->mem_type = PICAM360_MEMORY_TYPE_PROCESS;

	pthread_mutex_lock(&lg_frame_mlock);
	{
		if (_this->recieved_framecount != _this->framecount) {
			printf("lost frame in sync : %d\n", _this->cam_num);
		}
		_this->recieved_framecount++;

		float max_diff = 0;
		for (int i = 0; i < MAX_CAM_NUM && lg_v4l2_capture_ary[i] != NULL;
				i++) {
			if (lg_v4l2_capture_ary[i] == _this) {
				continue;
			}
			if (lg_v4l2_capture_ary[i]->recieved_framecount
					== lg_v4l2_capture_ary[i]->framecount) {
				max_diff = INT_MAX;
				break;
			}
			int _cur = (lg_v4l2_capture_ary[i]->recieved_framecount - 1)
					% BUFFER_NUM;
			PICAM360_IMAGE_T *_frame =
					&lg_v4l2_capture_ary[i]->frame_buffers[_cur];
			struct timeval diff;
			timersub(&_frame->timestamp, &frame->timestamp, &diff);
			float diff_sec = (float) diff.tv_sec
					+ (float) diff.tv_usec / 1000000;
			diff_sec = abs(diff_sec);
			if (diff_sec > max_diff) {
				max_diff = diff_sec;
			}
		}
		if (0 <= max_diff && max_diff < (1.0 / lg_fps / 2)) { //half frame
			for (int i = 0; i < MAX_CAM_NUM && lg_v4l2_capture_ary[i] != NULL;
					i++) {
				lg_v4l2_capture_ary[i]->framecount =
						lg_v4l2_capture_ary[i]->recieved_framecount;
				mrevent_trigger(&lg_v4l2_capture_ary[i]->frame_ready);
			}
		}
	}
	pthread_mutex_unlock(&lg_frame_mlock);

	{
		static int count = 0;
		static float elapsed_sec = 0;
		float _elapsed_sec;
		struct timeval diff;

		struct timeval now;
		gettimeofday(&now, NULL);

		timersub(&now, &timestamp, &diff);
		_elapsed_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
		if ((count % 200) == 0) {
			printf("v4l2_progress_image : %f\n", _elapsed_sec);
		}
		count++;
	}

	return _this->run ? 1 : 0;
}

static void* streaming_thread_fnc(void *arg) {
	v4l2_capture *_this = (v4l2_capture*) arg;

	handle_v4l2(_this->vstream_filepath, BUFFER_NUM, _this->cam_width,
			_this->cam_height, _this->cam_fps, v4l2_progress_image,
			(void*) _this);

	return NULL;
}

static void start(void *obj) {
	v4l2_capture *_this = (v4l2_capture*) obj;

	if (lg_v4l2_capture_ary[_this->cam_num]) {
		return;
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}

	strncpy(_this->vstream_filepath, lg_devicefiles[_this->cam_num],
			sizeof(_this->vstream_filepath));
	_this->cam_width = lg_width;
	_this->cam_height = lg_height;
	_this->cam_fps = lg_fps;
	_this->skipframe = lg_skipframe;

	_this->run = true;

	lg_v4l2_capture_ary[_this->cam_num] = _this;

	if (lg_mjpeg_kbps) {
		int ret;
		char cmd[512];
		sprintf(cmd,
				"plugins/v4l2_capture/Linux_UVC_TestAP/H264_UVC_TestAP --xuset-mjb %d %s",
				lg_mjpeg_kbps * 1000, lg_devicefiles[_this->cam_num]);
		printf("shell:%s\n", cmd);
		ret = system(cmd);
	}
	if (lg_v4l2_ctls) {
		for (int i = 0; lg_v4l2_ctls[i] != NULL; i++) {
			set_v4l2_ctl(lg_v4l2_ctls[i]->name, lg_v4l2_ctls[i]->value,
					_this->cam_num);
		}
	}

	pthread_create(&_this->streaming_thread, NULL, streaming_thread_fnc,
			(void*) _this);
}

static void stop(void *obj) {
	v4l2_capture *_this = (v4l2_capture*) obj;

	if(_this->run) {
		_this->run = false;
		pthread_join(_this->streaming_thread, NULL);
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static int set_param(void *obj, const char *param, const char *value_str) {
	v4l2_capture *_this = (v4l2_capture*) obj;
	if (strcmp(param, "cam_num") == 0) {
		int value = 0;
		sscanf(value_str, "%d", &value);
		_this->cam_num = MAX(0, MIN(value, MAX_CAM_NUM));
	}
}

static int get_param(void *obj, const char *param, char *value,
		int size) {

}

static int get_image(void *obj, PICAM360_IMAGE_T **image_p, int *num_p,
		int wait_usec) {
	v4l2_capture *_this = (v4l2_capture*) obj;

	int res = mrevent_wait(&_this->frame_ready, wait_usec);
	if (res != 0) {
		return -1;
	} else {
		mrevent_reset(&_this->frame_ready);
	}

	if(((_this->framecount - 1) % (_this->skipframe + 1)) != 0){
		return -1;
	}

	int cur = (_this->framecount - 1) % BUFFER_NUM;
	PICAM360_IMAGE_T *frame = &_this->frame_buffers[cur];

	*image_p = frame;
	*num_p = 1;

	return 0;
}

static void release(void *obj) {
	v4l2_capture *_this = (v4l2_capture*) obj;

	_this->super.stop(&_this->super);

	for (int i = 0; i < MAX_CAM_NUM; i++) {
		if (lg_v4l2_capture_ary[i] == _this) {
			lg_v4l2_capture_ary[i] = NULL;
		}
	}
	free(obj);
}

static void create_capture(void *user_data, VSTREAMER_T **out_capture) {
	VSTREAMER_T *capture = (VSTREAMER_T*) malloc(sizeof(v4l2_capture));
	memset(capture, 0, sizeof(v4l2_capture));
	strcpy(capture->name, CAPTURE_NAME);
	capture->release = release;
	capture->start = start;
	capture->stop = stop;
	capture->set_param = set_param;
	capture->get_param = get_param;
	capture->get_image = get_image;
	capture->user_data = capture;

	v4l2_capture *_private = (v4l2_capture*) capture;
	mrevent_init(&_private->frame_ready);

	if (out_capture) {
		*out_capture = capture;
	}
}

static int command_handler(void *user_data, const char *_buff) {
	int opt;
	int ret = 0;
	char buff[256] = { };
	strncpy(buff, _buff, sizeof(buff) - 1);
	char *cmd = strtok(buff, " \n");
	if (strncmp(cmd, PLUGIN_NAME ".add_v4l2_ctl", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			char *name = param;
			char *value_str = strtok(param, "=");
			if (name != NULL && value_str != NULL) {
				int value = 0;
				int num = sscanf(value_str, "%d", &value);
				for (int i = 0; lg_v4l2_ctls[i] != NULL; i++) {
					if (strcmp(lg_v4l2_ctls[i]->name, name) == 0) {
						lg_v4l2_ctls[i]->value += value;
						for (int j = 0; j < MAX_CAM_NUM; j++) {
							set_v4l2_ctl(lg_v4l2_ctls[i]->name,
									lg_v4l2_ctls[i]->value, j);
						}
						break;
					}
				}
			}

			printf("add_v4l2_ctl : completed\n");
		}
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id,
		uint32_t event_id) {
}

static void init_options(void *user_data, json_t *options) {
	{
		json_t *v4l2_ctls = json_object_get(options, PLUGIN_NAME ".v4l2_ctls");
		const char *key;
		json_t *value;

		json_object_foreach(v4l2_ctls, key, value)
		{
			V4l2_CTL_T *ctl = (V4l2_CTL_T*) malloc(sizeof(V4l2_CTL_T));
			strncpy(ctl->name, key, sizeof(ctl->name));
			ctl->value = json_number_value(value);
			add_v4l2_ctl(ctl);
		}
	}
	{
		json_t *value = json_object_get(options, PLUGIN_NAME ".width");
		if (value) {
			lg_width = json_number_value(value);
		}
	}
	{
		json_t *value = json_object_get(options, PLUGIN_NAME ".height");
		if (value) {
			lg_height = json_number_value(value);
		}
	}
	{
		json_t *value = json_object_get(options, PLUGIN_NAME ".fps");
		if (value) {
			lg_fps = json_number_value(value);
		}
	}
	{
		json_t *value = json_object_get(options, PLUGIN_NAME ".skipframe");
		if (value) {
			lg_skipframe = json_number_value(value);
		}
	}
	{
		json_t *value = json_object_get(options, PLUGIN_NAME ".mjpeg_kbps");
		if (value) {
			lg_mjpeg_kbps = json_number_value(value);
		}
	}
	for (int i = 0; i < MAX_CAM_NUM; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".cam%d_devicefile", i);
		json_t *value = json_object_get(options, buff);
		if (value) {
			int len = json_string_length(value);
			if (len < sizeof(lg_devicefiles[i])) {
				strncpy(lg_devicefiles[i], json_string_value(value), len);
			}
		}
	}
}

static void save_options(void *user_data, json_t *options) {
	for (int i = 0; i < MAX_CAM_NUM; i++) {
		char buff[256];
		if (lg_devicefiles[i][0] != 0) {
			sprintf(buff, PLUGIN_NAME ".cam%d_devicefile", i);
			json_object_set_new(options, buff, json_string(lg_devicefiles[i]));
		}
	}
	{
		json_object_set_new(options, PLUGIN_NAME ".width", json_real(lg_width));
		json_object_set_new(options, PLUGIN_NAME ".height",
				json_real(lg_height));
		json_object_set_new(options, PLUGIN_NAME ".fps", json_real(lg_fps));
	}
	if (lg_skipframe) {
		json_object_set_new(options, PLUGIN_NAME ".skipframe",
				json_real(lg_skipframe));
	}
	if (lg_mjpeg_kbps) {
		json_object_set_new(options, PLUGIN_NAME ".mjpeg_kbps",
				json_real(lg_mjpeg_kbps));
	}
	if (lg_v4l2_ctls) {
		json_t *v4l2_ctls = json_object();
		for (int i = 0; lg_v4l2_ctls[i] != NULL; i++) {
			json_object_set(v4l2_ctls, lg_v4l2_ctls[i]->name,
					json_real(lg_v4l2_ctls[i]->value));
		}
		json_object_set_new(options, PLUGIN_NAME ".v4l2_ctls", v4l2_ctls);
	}
}

void create_plugin(PLUGIN_HOST_T *plugin_host,
		PLUGIN_T **_plugin) {
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
		strcpy(factory->name, CAPTURE_NAME);
		factory->release = release;
		factory->create_vstreamer = create_capture;

		lg_plugin_host->add_vstreamer_factory(factory);
	}
}
