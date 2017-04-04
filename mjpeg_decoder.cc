/**
 * picam360-driver @ picam360 project
 */

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
#include <list>

#include "bcm_host.h"
#include "ilclient.h"

#include "mjpeg_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "mrevent.h"

#ifdef __cplusplus
}
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define NUM_OF_CAM 2

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
	std::list<_PACKET_T *> packets;
	pthread_mutex_t packets_mlock;
	MREVENT_T packet_ready;
};
class _SENDFRAME_ARG_T {
public:
	_SENDFRAME_ARG_T() {
		cam_run = false;
		cam_num = 0;
		framecount = 0;
		fps = 0;
		frameskip = 0;
		pthread_mutex_init(&frames_mlock, NULL);
		mrevent_init(&frame_ready);
		active_frame = NULL;
	}
	bool cam_run;
	int cam_num;
	int framecount;
	float fps;
	int frameskip;
	std::list<_FRAME_T *> frames;
	pthread_mutex_t frames_mlock;
	MREVENT_T frame_ready;
	pthread_t cam_thread;
	_FRAME_T *active_frame;
	void *user_data;
};

_SENDFRAME_ARG_T *lg_send_frame_arg[NUM_OF_CAM] = { };

static OMX_BUFFERHEADERTYPE* lg_egl_buffer[2] = { };
static COMPONENT_T* lg_egl_render[2] = { };

static void my_fill_buffer_done(void* data, COMPONENT_T* comp) {
	int index = (int) data;

	if (OMX_FillThisBuffer(ilclient_get_handle(lg_egl_render[index]),
			lg_egl_buffer[index]) != OMX_ErrorNone) {
		printf("test  OMX_FillThisBuffer failed in callback\n");
		exit(1);
	}
}

static void *sendframe_thread_func(void* arg) {
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) arg;
	int last_framecount = send_frame_arg->framecount;
	struct timeval last_time = { };
	gettimeofday(&last_time, NULL);

	int index = send_frame_arg->cam_num;

	OMX_VIDEO_PARAM_PORTFORMATTYPE format;
	COMPONENT_T *video_decode = NULL;
	COMPONENT_T *list[3];
	TUNNEL_T tunnel[2];
	ILCLIENT_T *client;
	int status = 0;
	unsigned int data_len = 0;

	memset(list, 0, sizeof(list));
	memset(tunnel, 0, sizeof(tunnel));

	if ((client = ilclient_init()) == NULL) {
		return (void *) -3;
	}

	if (OMX_Init() != OMX_ErrorNone) {
		ilclient_destroy(client);
		return (void *) -4;
	}

	// callback
	ilclient_set_fill_buffer_done_callback(client, my_fill_buffer_done,
			(void*) index);

	// create video_decode
	if (ilclient_create_component(client, &video_decode, "video_decode",
			ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_INPUT_BUFFERS) != 0)
		status = -14;
	list[0] = video_decode;

	// create lg_egl_render
	if (status == 0
			&& ilclient_create_component(client, &lg_egl_render[index],
					"lg_egl_render",
					ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_OUTPUT_BUFFERS)
					!= 0)
		status = -14;
	list[1] = lg_egl_render[index];

	set_tunnel(tunnel, video_decode, 131, lg_egl_render[index], 220);

	if (status == 0)
		ilclient_change_component_state(video_decode, OMX_StateIdle);

	memset(&format, 0, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
	format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
	format.nVersion.nVersion = OMX_VERSION;
	format.nPortIndex = 130;
	format.eCompressionFormat = OMX_VIDEO_CodingMJPEG;

	if (status == 0
			&& OMX_SetParameter(ILC_GET_HANDLE(video_decode),
					OMX_IndexParamVideoPortFormat, &format) == OMX_ErrorNone
			&& ilclient_enable_port_buffers(video_decode, 130, NULL, NULL, NULL)
					== 0) {
		OMX_BUFFERHEADERTYPE *buf;
		int port_settings_changed = 0;
		int first_packet = 1;

		ilclient_change_component_state(video_decode, OMX_StateExecuting);

		printf("milestone\n");

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
					float diff_sec = (float) diff.tv_sec
							+ (float) diff.tv_usec / 1000000;
					if (diff_sec > 1.0) {
						float tmp = (float) (send_frame_arg->framecount
								- last_framecount) / diff_sec;
						float w = diff_sec / 10;
						send_frame_arg->fps = send_frame_arg->fps * (1.0 - w)
								+ tmp * w;

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

				buf = ilclient_get_input_buffer(video_decode, 130, 1);

				data_len = MIN(buf->nAllocLen, packet->len);
				memcpy(buf->pBuffer, packet->data, data_len);

				if (port_settings_changed == 0
						&& ((data_len > 0
								&& ilclient_remove_event(video_decode,
										OMX_EventPortSettingsChanged, 131, 0, 0,
										1) == 0)
								|| (data_len == 0
										&& ilclient_wait_for_event(video_decode,
												OMX_EventPortSettingsChanged,
												131, 0, 0, 1,
												ILCLIENT_EVENT_ERROR
														| ILCLIENT_PARAMETER_CHANGED,
												10000) == 0))) {
					port_settings_changed = 1;

					if (ilclient_setup_tunnel(tunnel, 0, 0) != 0) {
						status = -7;
						break;
					}

					// Set lg_egl_render to idle
					ilclient_change_component_state(lg_egl_render[index],
							OMX_StateIdle);

					// Enable the output port and tell lg_egl_render to use the texture as a buffer
					//ilclient_enable_port(lg_egl_render, 221); THIS BLOCKS SO CAN'T BE USED
					if (OMX_SendCommand(ILC_GET_HANDLE(lg_egl_render[index]),
							OMX_CommandPortEnable, 221, NULL)
							!= OMX_ErrorNone) {
						printf("OMX_CommandPortEnable failed.\n");
						exit(1);
					}

					if (OMX_UseEGLImage(ILC_GET_HANDLE(lg_egl_render[index]),
							&lg_egl_buffer[index], 221, NULL,
							send_frame_arg->user_data) != OMX_ErrorNone) {
						printf("OMX_UseEGLImage failed.\n");
						exit(1);
					}

					// Set lg_egl_render to executing
					ilclient_change_component_state(lg_egl_render[index],
							OMX_StateExecuting);

					// Request lg_egl_render to write data to the texture buffer
					if (OMX_FillThisBuffer(ILC_GET_HANDLE(lg_egl_render[index]),
							lg_egl_buffer[index]) != OMX_ErrorNone) {
						printf("OMX_FillThisBuffer failed.\n");
						exit(1);
					}
				}
				if (!data_len) {
					break;
				}
				buf->nFilledLen = data_len;

				buf->nOffset = 0;
				if (first_packet) {
					buf->nFlags = OMX_BUFFERFLAG_STARTTIME;
					first_packet = 0;
				} else {
					buf->nFlags = OMX_BUFFERFLAG_TIME_UNKNOWN;
				}

				if (packet->eof) {
					buf->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;
				}

				if (OMX_EmptyThisBuffer(ILC_GET_HANDLE(video_decode), buf)
						!= OMX_ErrorNone) {
					status = -6;
					break;
				}

				if (packet->eof) {
					delete packet;
					break;
				} else {
					delete packet;
				}
			}
			delete frame;
		}

		buf->nFilledLen = 0;
		buf->nFlags = OMX_BUFFERFLAG_TIME_UNKNOWN | OMX_BUFFERFLAG_EOS;

		if (OMX_EmptyThisBuffer(ILC_GET_HANDLE(video_decode), buf)
				!= OMX_ErrorNone)
			status = -20;

		// need to flush the renderer to allow video_decode to disable its input port
		ilclient_flush_tunnels(tunnel, 0);

		ilclient_disable_port_buffers(video_decode, 130, NULL, NULL, NULL);
	}

	ilclient_disable_tunnel(tunnel);
	ilclient_teardown_tunnels(tunnel);

	ilclient_state_transition(list, OMX_StateIdle);
	ilclient_state_transition(list, OMX_StateLoaded);

	ilclient_cleanup_components(list);

	OMX_Deinit();

	ilclient_destroy(client);
	return (void *) status;
}

void mjpeg_decode(int cam_num, unsigned char *data, int data_len) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1));
	_SENDFRAME_ARG_T *send_frame_arg = lg_send_frame_arg[cam_num];
	if (send_frame_arg->active_frame == NULL) {
		if (data[0] == 0xD8 && data[1] == 0xD8) { //SOI
			send_frame_arg->active_frame = new _FRAME_T;

			pthread_mutex_lock(&send_frame_arg->frames_mlock);
			send_frame_arg->frames.push_back(active_frame);
			pthread_mutex_unlock(&send_frame_arg->frames_mlock);
			mrevent_trigger(&send_frame_arg->frame_ready);
		}
	}
	if (send_frame_arg->active_frame != NULL) {
		_PACKET_T *packet = new _PACKET_T;
		{
			packet->len = data_len;
			if (packet->len > RTP_MAXPAYLOADSIZE) {
				fprintf(stderr, "packet length exceeded. %d\n", packet->len);
				packet->len = RTP_MAXPAYLOADSIZE;
			}
			memcpy(packet->data, data, packet->len);
		}
		if (data[data_len - 2] == 0xD8 && && data[data_len - 1] == 0xD9) { //EOI
			packet->eof = true;

			pthread_mutex_lock(&active_frame->packets_mlock);
			active_frame->packets.push_back(packet);
			mrevent_trigger(&active_frame->packet_ready);
			pthread_mutex_unlock(&active_frame->packets_mlock);

			active_frame = NULL;
		} else {
			pthread_mutex_lock(&active_frame->packets_mlock);
			active_frame->packets.push_back(packet);
			pthread_mutex_unlock(&active_frame->packets_mlock);
			mrevent_trigger(&active_frame->packet_ready);
		}
	}

}
void init_mjpeg_decoder(int cam_num, void *user_data) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1));
	if (lg_send_frame_arg[cam_num]) {
		return;
	}
	lg_send_frame_arg[cam_num] = new _SENDFRAME_ARG_T;
	lg_send_frame_arg[cam_num]->cam_num = cam_num;
	lg_send_frame_arg[cam_num]->user_data = user_data;

	lg_send_frame_arg[cam_num]->cam_run = true;
	pthread_create(&lg_send_frame_arg[cam_num]->cam_thread, NULL,
			sendframe_thread_func, (void*) lg_send_frame_arg[cam_num]);
}
void deinit_mjpeg_decoder(int cam_num) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1));
	if (!lg_send_frame_arg[cam_num]) {
		return;
	}

	lg_send_frame_arg[cam_num]->cam_run = false;
	pthread_join(lg_send_frame_arg[cam_num]->cam_thread, NULL);

	delete lg_send_frame_arg[cam_num];
	lg_send_frame_arg[cam_num] = NULL;
}

float mjpeg_decoder_get_fps(int cam_num) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1));
	if (!lg_send_frame_arg[cam_num]) {
		return 0;
	}
	return lg_send_frame_arg[cam_num]->fps;
}
int mjpeg_decoder_get_frameskip(int cam_num) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1));
	if (!lg_send_frame_arg[cam_num]) {
		return 0;
	}
	return lg_send_frame_arg[cam_num]->frameskip;
}
