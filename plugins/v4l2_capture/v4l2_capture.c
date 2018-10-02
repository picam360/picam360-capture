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
#if __linux
#include <sys/prctl.h>
#endif

#ifdef USE_GLES
#include "GLES2/gl2.h"
#include "GLES2/gl2ext.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#include "nvbuf_utils.h"
#else
#include <GL/glew.h>
#include <GLFW/glfw3.h>
//#include "GL/gl.h"
//#include "GL/glut.h"
//#include "GL/glext.h"
#endif

#include <drm/drm_fourcc.h>

#include "v4l2_capture.h"
#include "v4l2_handler.h"

#define PLUGIN_NAME "v4l2_capture"
#define CAPTURE_NAME "v4l2_capture"

static PLUGIN_HOST_T *lg_plugin_host = NULL;
static int lg_width = 2048;
static int lg_height = 1536;
static int lg_fps = 15;
static char *lg_devices[3] = { //
		"/dev/video0", /**/
		"/dev/video1", /**/
		"/dev/video2", /**/
		};

typedef struct _V4l2_CTL_T {
	char name[256];
	int value;
} V4l2_CTL_T;

typedef struct _v4l2_capture {
	CAPTURE_T super;

	int cam_num;
	uint32_t frame_num;

	pthread_t v4l2_thread;
	ENCODER_STREAM_CALLBACK callback;
	void *user_data;
} v4l2_capture;

static void set_v4l2_ctl(const char *name, const int value) {
	for (int i = 0; i < CAMERA_NUM; i++) {
		if (state->options.v4l2_devicefile[i][0] != '\0') {
			char cmd[256];
			sprintf(cmd, "v4l2-ctl --set-ctrl=%s=%d -d %s", name, value, state->options.v4l2_devicefile[i]);
			system(cmd);
		}
	}
}

static void add_v4l2_ctl(V4l2_CTL_T *ctl) {
	if (state->v4l2_ctls == NULL) {
		const int INITIAL_SPACE = 16;
		state->v4l2_ctls = malloc(sizeof(V4l2_CTL_T*) * INITIAL_SPACE);
		memset(state->v4l2_ctls, 0, sizeof(V4l2_CTL_T*) * INITIAL_SPACE);
		state->v4l2_ctls[INITIAL_SPACE - 1] = (void*) -1;
	}

	for (int i = 0; state->v4l2_ctls[i] != (void*) -1; i++) {
		if (state->v4l2_ctls[i] == NULL) {
			state->v4l2_ctls[i] = ctl;
			return;
		}
		if (state->v4l2_ctls[i + 1] == (void*) -1) {
			int space = (i + 2) * 2;
			if (space > 256) {
				fprintf(stderr, "error on add_mpu\n");
				return;
			}
			V4l2_CTL_T **current = state->v4l2_ctls;
			state->v4l2_ctls = malloc(sizeof(V4l2_CTL_T*) * space);
			memcpy(state->v4l2_ctls, current, sizeof(V4l2_CTL_T*) * (i + 1));
			state->v4l2_ctls[space - 1] = (void*) -1;
			free(current);
		}
	}
}

static int v4l2_progress_image(const void *p, int size, void *user_data) {
	v4l2_capture *send_frame_arg = (v4l2_capture*) arg;

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
				if (lg_video_mjpeg_xmp_callback) { //xmp injection
					int xmp_len = lg_video_mjpeg_xmp_callback(packet->data + 2,
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

	return 1; //true
}

static void *v4l2_thread_func(void* arg) {
	v4l2_capture *_this = (v4l2_capture*) arg;

	handle_v4l2(lg_devices[_this->cam_num], lg_width, lg_height, lg_fps, _this->n_buffers, _this->dma_fds, v4l2_progress_image, arg);

	return NULL;
}

static void start(void *obj, int cam_num, void *display, void *context, int egl_image_num) {
	v4l2_capture *_this = (v4l2_capture*) obj;
	_this->cam_num = cam_num;
	pthread_create(&_this->v4l2_thread, NULL, v4l2_thread_func, (void*) _this);
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
	if (strncmp(cmd, PLUGIN_NAME ".add_v4l2_ctl", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			char name[256] = { };
			int value = 0;
			int num = sscanf(param, "%255[^=]=%d", name, &value);
			if (num == 2) {
				for (int i = 0; state->v4l2_ctls[i] != NULL; i++) {
					if (strcmp(state->v4l2_ctls[i]->name, name) == 0) {
						state->v4l2_ctls[i]->value += value;
						set_v4l2_ctl(state->v4l2_ctls[i]->name, state->v4l2_ctls[i]->value);
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
		json_t *v4l2_ctls = json_object_get(options, "v4l2_ctls");
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

	if (state->v4l2_ctls) {
		for (int i = 0; state->v4l2_ctls[i] != NULL; i++) {
			set_v4l2_ctl(state->v4l2_ctls[i]->name, state->v4l2_ctls[i]->value);
		}
	}
}

static void save_options(void *user_data, json_t *options) {
	if (state->v4l2_ctls) {
		json_t *v4l2_ctls = json_object();
		for (int i = 0; state->v4l2_ctls[i] != NULL; i++) {
			json_object_set(v4l2_ctls, state->v4l2_ctls[i]->name, json_real(state->v4l2_ctls[i]->value));
		}
		json_object_set_new(options, "v4l2_ctls", v4l2_ctls);
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
