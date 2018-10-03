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
#include <list>
#if __linux
#include <sys/prctl.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "mrevent.h"
#include "v4l2_handler.h"
#include "v4l2_capture.h"

#ifdef __cplusplus
}
#endif

#define PLUGIN_NAME "v4l2_capture"
#define CAPTURE_NAME "v4l2_capture"
#define CAMERA_NUM 2

static PLUGIN_HOST_T *lg_plugin_host = NULL;
static int lg_width = 2048;
static int lg_height = 1536;
static int lg_fps = 15;
static char lg_devicefiles[CAMERA_NUM][256] = { };

class _PACKET_T {
public:
	_PACKET_T() {
		len = 0;
		eof = false;
	}
	int len;
	char data[RTP_MAXPAYLOADSIZE];
	bool eof;
};

class _FRAME_T {
public:
	_FRAME_T() {
		num_of_bytes = 0;
		pthread_mutex_init(&packets_mlock, NULL);
		mrevent_init(&packet_ready);
	}
	~_FRAME_T() {
		_FRAME_T *frame = this;
		while (!frame->packets.empty()) {
			_PACKET_T *packet;
			pthread_mutex_lock(&frame->packets_mlock);
			packet = *(frame->packets.begin());
			frame->packets.pop_front();
			if (frame->packets.empty()) {
				mrevent_reset(&frame->packet_ready);
			}
			pthread_mutex_unlock(&frame->packets_mlock);
			delete packet;
		}
	}
	int num_of_bytes;
	std::list<_PACKET_T *> packets;
	pthread_mutex_t packets_mlock;
	MREVENT_T packet_ready;
};

class _SENDFRAME_ARG_T {
public:
	_SENDFRAME_ARG_T() {
		memset(vstream_filepath, 0, sizeof(vstream_filepath));
		cam_run = false;
		cam_num = 0;
		cam_width = 0;
		cam_height = 0;
		cam_fps = 0;
		rtp = NULL;
		skip_frame = 0;
		framecount = 0;
		recieved_framecount = 0;
		fps = 0;
		frameskip = 0;
		pthread_mutex_init(&frames_mlock, NULL);
		mrevent_init(&frame_ready);
	}
	char vstream_filepath[256];
	bool cam_run;
	int cam_num;
	int cam_width;
	int cam_height;
	int cam_fps;
	RTP_T *rtp;
	int skip_frame;
	unsigned int framecount;
	unsigned int recieved_framecount;
	float fps;
	int frameskip;
	std::list<_FRAME_T *> frames;
	pthread_mutex_t frames_mlock;
	MREVENT_T frame_ready;
	pthread_t cam_thread;
	void *user_data;
};

typedef struct _V4l2_CTL_T {
	char name[256];
	int value;
} V4l2_CTL_T;

V4l2_CTL_T **lg_v4l2_ctls = NULL;

typedef struct _v4l2_capture {
	CAPTURE_T super;

	_SENDFRAME_ARG_T *send_frame_arg;
} v4l2_capture;

static void set_v4l2_ctl(const char *name, const int value) {
	for (int i = 0; i < CAMERA_NUM; i++) {
		if (lg_devicefiles[i][0] != '\0') {
			char cmd[256];
			sprintf(cmd, "v4l2-ctl --set-ctrl=%s=%d -d %s", name, value, lg_devicefiles[i]);
			system(cmd);
		}
	}
}

static void add_v4l2_ctl(V4l2_CTL_T *ctl) {
	if (lg_v4l2_ctls == NULL) {
		const int INITIAL_SPACE = 16;
		lg_v4l2_ctls = malloc(sizeof(V4l2_CTL_T*) * INITIAL_SPACE);
		memset(lg_v4l2_ctls, 0, sizeof(V4l2_CTL_T*) * INITIAL_SPACE);
		lg_v4l2_ctls[INITIAL_SPACE - 1] = (void*) -1;
	}

	for (int i = 0; lg_v4l2_ctls[i] != (void*) -1; i++) {
		if (lg_v4l2_ctls[i] == NULL) {
			lg_v4l2_ctls[i] = ctl;
			return;
		}
		if (lg_v4l2_ctls[i + 1] == (void*) -1) {
			int space = (i + 2) * 2;
			if (space > 256) {
				fprintf(stderr, "error on add_mpu\n");
				return;
			}
			V4l2_CTL_T **current = lg_v4l2_ctls;
			lg_v4l2_ctls = malloc(sizeof(V4l2_CTL_T*) * space);
			memcpy(lg_v4l2_ctls, current, sizeof(V4l2_CTL_T*) * (i + 1));
			lg_v4l2_ctls[space - 1] = (void*) -1;
			free(current);
		}
	}
}

static void *sendframe_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "SEND");
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) arg;
	int last_framecount = send_frame_arg->framecount;
	struct timeval last_time = { };
	gettimeofday(&last_time, NULL);
	while (send_frame_arg->cam_run) {
		int res = mrevent_wait(&send_frame_arg->frame_ready, 100 * 1000);
		if (res != 0) {
			continue;
		}
		_FRAME_T *frame;
		pthread_mutex_lock(&send_frame_arg->frames_mlock);
		while (1) {
			frame = *(send_frame_arg->frames.begin());
			send_frame_arg->frames.pop_front();
			send_frame_arg->framecount++;
			if (send_frame_arg->frames.empty()) {
				mrevent_reset(&send_frame_arg->frame_ready);
				break;
			}
			send_frame_arg->frameskip++;
			delete frame; //skip frame
		}
		pthread_mutex_unlock(&send_frame_arg->frames_mlock);
		while (send_frame_arg->cam_run) {
			{ //fps
				struct timeval time = { };
				gettimeofday(&time, NULL);

				struct timeval diff;
				timersub(&time, &last_time, &diff);
				float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
				if (diff_sec > 1.0) {
					float tmp = (float) (send_frame_arg->framecount - last_framecount) / diff_sec;
					float w = diff_sec / 10;
					send_frame_arg->fps = send_frame_arg->fps * (1.0 - w) + tmp * w;

					last_framecount = send_frame_arg->framecount;
					last_time = time;
				}
			}
			int res = mrevent_wait(&frame->packet_ready, 100 * 1000);
			if (res != 0) {
				continue;
			}
			_PACKET_T *packet = NULL;
			pthread_mutex_lock(&frame->packets_mlock);
			if (!frame->packets.empty()) {
				packet = *(frame->packets.begin());
				frame->packets.pop_front();
			}
			if (frame->packets.empty()) {
				mrevent_reset(&frame->packet_ready);
			}
			pthread_mutex_unlock(&frame->packets_mlock);
			if (packet == NULL) {
				fprintf(stderr, "packet is null\n");
				continue;
			}
			// send the packet
			lg_plugin_host->decode_video(send_frame_arg->cam_num, (unsigned char*) packet->data, packet->len);
			if (packet->eof) {
				delete packet;
				break;
			} else {
				delete packet;
			}
		}
		delete frame;
	}
	return NULL;
}

static int v4l2_progress_image(const void *p, int size, void *arg) {
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) arg;

	//remove space
	for (; size > 0;) {
		if (((unsigned char*) p)[size - 1] == 0xD9) {
			break;
		} else {
			size--;
		}
	}

	if ((send_frame_arg->recieved_framecount++ % (send_frame_arg->skip_frame + 1)) == 0 && size != 0) {
		_FRAME_T *active_frame = new _FRAME_T;

		pthread_mutex_lock(&send_frame_arg->frames_mlock);
		send_frame_arg->frames.push_back(active_frame);
		pthread_mutex_unlock(&send_frame_arg->frames_mlock);
		mrevent_trigger(&send_frame_arg->frame_ready);

		for (int i = 0; i < size;) {
			_PACKET_T *packet = new _PACKET_T;
			if (i == 0) {
				i += 2; //increment soi marker
				packet->len = 2;
				packet->data[0] = 0xFF;
				packet->data[1] = 0xD8; //soi marker
				{ //xmp injection
					int xmp_len = lg_plugin_host->xmp(packet->data + 2,
					RTP_MAXPAYLOADSIZE - 2, send_frame_arg->cam_num);
					if (packet->len + xmp_len <= RTP_MAXPAYLOADSIZE) {
						packet->len += xmp_len;
					}
				}
			}
			int len;
			if (packet->len + (size - i) > RTP_MAXPAYLOADSIZE) {
				len = RTP_MAXPAYLOADSIZE - packet->len;
			} else {
				len = (size - i);
			}
			if (len) {
				memcpy(packet->data + packet->len, (unsigned char*) p + i, len);
				packet->len += len;
				i += len;
			}
			pthread_mutex_lock(&active_frame->packets_mlock);
			active_frame->num_of_bytes += packet->len;
			if (i == size) { // active_frame->num_of_bytes is size + xmp_len
				packet->eof = true;
			}
			active_frame->packets.push_back(packet);
			mrevent_trigger(&active_frame->packet_ready);
			pthread_mutex_unlock(&active_frame->packets_mlock);
		}
	}

	return send_frame_arg->cam_run ? 1 : 0;
}

static void *camx_thread_func_v4l2(void* arg) {
	pthread_setname_np(pthread_self(), "CAM");
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) arg;

	pthread_t sendframe_thread;
	pthread_create(&sendframe_thread, NULL, sendframe_thread_func, arg);

	handle_v4l2(send_frame_arg->vstream_filepath, send_frame_arg->cam_width, send_frame_arg->cam_height, send_frame_arg->cam_fps, v4l2_progress_image, arg);

	return NULL;
}

static void start(void *obj, int cam_num, void *display, void *context, int egl_image_num) {
	v4l2_capture *_this = (v4l2_capture*) obj;

	_SENDFRAME_ARG_T *send_frame_arg = new _SENDFRAME_ARG_T;
	send_frame_arg->cam_num = cam_num;
	strncpy(send_frame_arg->vstream_filepath, lg_devicefiles[cam_num], sizeof(send_frame_arg->vstream_filepath));
	send_frame_arg->cam_width = lg_width;
	send_frame_arg->cam_height = lg_height;
	send_frame_arg->cam_fps = lg_fps;

	send_frame_arg->cam_run = true;

	pthread_create(&send_frame_arg->cam_thread, NULL, camx_thread_func_v4l2, (void*) send_frame_arg);

	_this->send_frame_arg = send_frame_arg;
}

static void release(void *obj) {
	v4l2_capture *_this = (v4l2_capture*) obj;
	free(obj);
}

static float get_fps(void *user_data) {
	return 0;
}
static void create_capture(void *user_data, CAPTURE_T **out_capture) {
	CAPTURE_T *capture = (CAPTURE_T*) malloc(sizeof(v4l2_capture));
	memset(capture, 0, sizeof(v4l2_capture));
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
	int opt;
	int ret = 0;
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd = strtok(buff, " \n");
	if (strncmp(cmd, PLUGIN_NAME ".add_v4l2_ctl", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			char name[256] = { };
			int value = 0;
			int num = sscanf(param, "%255[^=]=%d", name, &value);
			if (num == 2) {
				for (int i = 0; lg_v4l2_ctls[i] != NULL; i++) {
					if (strcmp(lg_v4l2_ctls[i]->name, name) == 0) {
						lg_v4l2_ctls[i]->value += value;
						set_v4l2_ctl(lg_v4l2_ctls[i]->name, lg_v4l2_ctls[i]->value);
						break;
					}
				}
			}

			printf("add_v4l2_ctl : completed\n");
		}
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
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
	for (int i = 0; i < CAMERA_NUM; i++) {
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

	if (lg_v4l2_ctls) {
		for (int i = 0; lg_v4l2_ctls[i] != NULL; i++) {
			set_v4l2_ctl(lg_v4l2_ctls[i]->name, lg_v4l2_ctls[i]->value);
		}
	}
}

static void save_options(void *user_data, json_t *options) {
	for (int i = 0; i < CAMERA_NUM; i++) {
		char buff[256];
		if (lg_devicefiles[i][0] != 0) {
			sprintf(buff, PLUGIN_NAME ".cam%d_devicefile", i);
			json_object_set_new(options, buff, json_string(lg_devicefiles[i]));
		}
	}
	if (lg_v4l2_ctls) {
		json_t *v4l2_ctls = json_object();
		for (int i = 0; lg_v4l2_ctls[i] != NULL; i++) {
			json_object_set(v4l2_ctls, lg_v4l2_ctls[i]->name, json_real(lg_v4l2_ctls[i]->value));
		}
		json_object_set_new(options, PLUGIN_NAME ".v4l2_ctls", v4l2_ctls);
	}
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
