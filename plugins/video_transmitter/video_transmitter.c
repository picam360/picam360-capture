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

#define PLUGIN_NAME "video_transmitter"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

typedef struct _video_transmitter {
	DECODER_T super;

	//nal
	bool in_nal;
	uint32_t nal_len;
	uint8_t nal_type;
	uint8_t *nal_buff;
	uint32_t nal_pos;

	void *user_data;
} video_transmitter;

static void init(void *obj, int cam_num, void *context, void *cam_texture, void **egl_images, int egl_image_num) {

}

static void release(void *obj) {
	video_transmitter *_this = (video_transmitter*) obj;
	free(obj);
}

static void decode(void *user_data, unsigned char *data, int data_len) {
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
			rtp_sendpacket(send_frame_arg->rtp, (unsigned char*) packet->data, packet->len,
			PT_CAM_BASE + send_frame_arg->cam_num);
			if (packet->eof) {
				rtp_flush(send_frame_arg->rtp);
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
