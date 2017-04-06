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

#include "mjpeg_decoder.h"

#define RTP_MAXPAYLOADSIZE (8*1024-12)

#ifdef __cplusplus
extern "C" {
#endif

#include "bcm_host.h"
#include "ilclient.h"
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
		xmp_info = false;
		memset(quatanion, 0, sizeof(quatanion));
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
	bool xmp_info;
	float quatanion[4];
};
class _SENDFRAME_ARG_T {
public:
	_SENDFRAME_ARG_T() {
		user_data = NULL;
		cam_run = false;
		cam_num = 0;
		framecount = 0;
		decodereqcount = 0;
		decodedcount = 0;
		fps = 0;
		frameskip = 0;
		pthread_mutex_init(&frames_mlock, NULL);
		mrevent_init(&frame_ready);
		mrevent_init(&buffer_ready);
		active_frame = NULL;
		memset(egl_buffer, 0, sizeof(egl_buffer));
		egl_render = NULL;
		xmp_info = false;
		memset(quatanion, 0, sizeof(quatanion));
	}
	void *user_data;
	bool cam_run;
	int cam_num;
	int framecount;
	int decodereqcount;
	int decodedcount;
	float fps;
	int frameskip;
	std::list<_FRAME_T *> frames;
	pthread_mutex_t frames_mlock;
	MREVENT_T frame_ready;
	MREVENT_T buffer_ready;
	pthread_t cam_thread;
	_FRAME_T *active_frame;
	OMX_BUFFERHEADERTYPE* egl_buffer[2];
	COMPONENT_T* egl_render;
	bool xmp_info;
	float quatanion[4];
};

static PLUGIN_HOST_T *lg_plugin_host = NULL;
static _SENDFRAME_ARG_T *lg_send_frame_arg[NUM_OF_CAM] = { };

static void my_fill_buffer_done(void* data, COMPONENT_T* comp) {
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) data;

	OMX_BUFFERHEADERTYPE *egl_buffer = ilclient_get_output_buffer(comp, 221, 1);
	printf("%d\n", (int) egl_buffer->pAppPrivate);
	if (lg_plugin_host) {
		lg_plugin_host->set_cam_texture_cur(send_frame_arg->cam_num,
				(int) egl_buffer->pAppPrivate);
		if (send_frame_arg->xmp_info) {
			lg_plugin_host->set_camera_quatanion(send_frame_arg->cam_num,
					send_frame_arg->quatanion);
		}
	}
	//int cam_num = send_frame_arg->cam_num;
	if (OMX_FillThisBuffer(ilclient_get_handle(send_frame_arg->egl_render),
			egl_buffer) != OMX_ErrorNone) {
		printf("test  OMX_FillThisBuffer failed in callback\n");
		exit(1);
	}
}

static void *sendframe_thread_func(void* arg) {
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) arg;
	int last_framecount = send_frame_arg->framecount;
	struct timeval last_time = { };
	gettimeofday(&last_time, NULL);

	int cam_num = send_frame_arg->cam_num;

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
			(void*) send_frame_arg);

	// create video_decode
	if (ilclient_create_component(client, &video_decode, (char*) "video_decode",
			(ILCLIENT_CREATE_FLAGS_T)(
					ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_INPUT_BUFFERS))
			!= 0)
		status = -14;
	list[0] = video_decode;

	// create lg_egl_render
	if (status == 0
			&& ilclient_create_component(client, &send_frame_arg->egl_render,
					(char*) "egl_render",
					(ILCLIENT_CREATE_FLAGS_T)(
							ILCLIENT_DISABLE_ALL_PORTS
									| ILCLIENT_ENABLE_OUTPUT_BUFFERS)) != 0)
		status = -14;
	list[1] = send_frame_arg->egl_render;

	set_tunnel(tunnel, video_decode, 131, send_frame_arg->egl_render, 220);

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

				data_len = MIN((int )buf->nAllocLen, packet->len);
				memcpy(buf->pBuffer, packet->data, data_len);

				if (port_settings_changed == 0
						&& ilclient_remove_event(video_decode,
								OMX_EventPortSettingsChanged, 131, 0, 0, 1)
								== 0) {
					port_settings_changed = 1;
					printf("port changed %d\n", cam_num);

					if (ilclient_setup_tunnel(tunnel, 0, 0) != 0) {
						status = -7;
						break;
					}

					// Set lg_egl_render to idle
					ilclient_change_component_state(send_frame_arg->egl_render,
							OMX_StateIdle);

					// Enable the output port and tell lg_egl_render to use the texture as a buffer
					//ilclient_enable_port(lg_egl_render, 221); THIS BLOCKS SO CAN'T BE USED
					if (OMX_SendCommand(
							ILC_GET_HANDLE(send_frame_arg->egl_render),
							OMX_CommandPortEnable, 221, NULL)
							!= OMX_ErrorNone) {
						printf("OMX_CommandPortEnable failed.\n");
						exit(1);
					}

					for (int i = 0; i < 2; i++) {
						OMX_STATETYPE state;
						OMX_GetState(ILC_GET_HANDLE(send_frame_arg->egl_render),
								&state);
						if (state != OMX_StateIdle) {
							if (state != OMX_StateLoaded) {
								ilclient_change_component_state(
										send_frame_arg->egl_render,
										OMX_StateLoaded);
							}
							ilclient_change_component_state(
									send_frame_arg->egl_render, OMX_StateIdle);
						}
						if (OMX_UseEGLImage(
								ILC_GET_HANDLE(send_frame_arg->egl_render),
								&send_frame_arg->egl_buffer[i], 221, (void*) i,
								((void**) send_frame_arg->user_data)[i])
								!= OMX_ErrorNone) {
							printf("OMX_UseEGLImage failed.\n");
							exit(1);
						}
					}

					// Set lg_egl_render to executing
					ilclient_change_component_state(send_frame_arg->egl_render,
							OMX_StateExecuting);

					// Request lg_egl_render to write data to the texture buffer
					for (int i = 0; i < 2; i++) {
						if (OMX_FillThisBuffer(
								ILC_GET_HANDLE(send_frame_arg->egl_render),
								send_frame_arg->egl_buffer[i])
								!= OMX_ErrorNone) {
							printf("OMX_FillThisBuffer failed.\n");
							exit(1);
						}
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
					send_frame_arg->xmp_info = frame->xmp_info;
					memcpy(send_frame_arg->quatanion, frame->quatanion,
							sizeof(send_frame_arg->quatanion));
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

static void parse_xml(char *xml, _FRAME_T *frame) {
	frame->xmp_info = true;

	char *q_str = NULL;
	q_str = strstr(xml, "<quaternion");
	if (q_str) {
		float quatanion[4];
		sscanf(q_str, "<quaternion w=\"%f\" x=\"%f\" y=\"%f\" z=\"%f\" />",
				&quatanion[0], &quatanion[1], &quatanion[2], &quatanion[3]);
		//convert from mpu coodinate to opengl coodinate
		frame->quatanion[0] = quatanion[1]; //x
		frame->quatanion[1] = quatanion[3]; //y : swap y and z
		frame->quatanion[2] = -quatanion[2]; //z : swap y and z
		frame->quatanion[3] = quatanion[0]; //w
	}
}

void mjpeg_decode(int cam_num, unsigned char *data, int data_len) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1), 0);
	if (!lg_send_frame_arg[cam_num]) {
		return;
	}
	_SENDFRAME_ARG_T *send_frame_arg = lg_send_frame_arg[cam_num];
	if (send_frame_arg->active_frame == NULL) {
		if (data[0] == 0xFF && data[1] == 0xD8) { //SOI
			send_frame_arg->active_frame = new _FRAME_T;

			pthread_mutex_lock(&send_frame_arg->frames_mlock);
			send_frame_arg->frames.push_back(send_frame_arg->active_frame);
			pthread_mutex_unlock(&send_frame_arg->frames_mlock);
			mrevent_trigger(&send_frame_arg->frame_ready);

			if (data[2] == 0xFF && data[3] == 0xE1) { //xmp
				int xmp_len = 0;
				xmp_len = ((unsigned char*) data)[4] << 8;
				xmp_len += ((unsigned char*) data)[5];

				char *xml = (char*) data + strlen((char*) data) + 1;
				parse_xml(xml, send_frame_arg->active_frame);
			}
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
		if (data[data_len - 2] == 0xFF && data[data_len - 1] == 0xD9) { //EOI
			packet->eof = true;

			pthread_mutex_lock(&send_frame_arg->active_frame->packets_mlock);
			send_frame_arg->active_frame->packets.push_back(packet);
			mrevent_trigger(&send_frame_arg->active_frame->packet_ready);
			pthread_mutex_unlock(&send_frame_arg->active_frame->packets_mlock);

			send_frame_arg->active_frame = NULL;
		} else {
			pthread_mutex_lock(&send_frame_arg->active_frame->packets_mlock);
			send_frame_arg->active_frame->packets.push_back(packet);
			pthread_mutex_unlock(&send_frame_arg->active_frame->packets_mlock);
			mrevent_trigger(&send_frame_arg->active_frame->packet_ready);
		}
	}

}
void init_mjpeg_decoder(PLUGIN_HOST_T *plugin_host, int cam_num,
		void *user_data) {
	lg_plugin_host = plugin_host;

	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1), 0);
	if (lg_send_frame_arg[cam_num]) {
		return;
	}
	lg_send_frame_arg[cam_num] = new _SENDFRAME_ARG_T;
	lg_send_frame_arg[cam_num]->cam_num = cam_num;
	lg_send_frame_arg[cam_num]->user_data = user_data;

	lg_send_frame_arg[cam_num]->cam_run = true;
	pthread_create(&lg_send_frame_arg[cam_num]->cam_thread, NULL,
			sendframe_thread_func, (void*) lg_send_frame_arg[cam_num]);
	char buff[256];
	sprintf(buff, "sendframe_thread_func:%d", cam_num);
	pthread_setname_np(lg_send_frame_arg[cam_num]->cam_thread, buff);
}
void deinit_mjpeg_decoder(int cam_num) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1), 0);
	if (!lg_send_frame_arg[cam_num]) {
		return;
	}

	lg_send_frame_arg[cam_num]->cam_run = false;
	pthread_join(lg_send_frame_arg[cam_num]->cam_thread, NULL);

	delete lg_send_frame_arg[cam_num];
	lg_send_frame_arg[cam_num] = NULL;
}

float mjpeg_decoder_get_fps(int cam_num) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1), 0);
	if (!lg_send_frame_arg[cam_num]) {
		return 0;
	}
	return lg_send_frame_arg[cam_num]->fps;
}
int mjpeg_decoder_get_frameskip(int cam_num) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1), 0);
	if (!lg_send_frame_arg[cam_num]) {
		return 0;
	}
	return lg_send_frame_arg[cam_num]->frameskip;
}

void mjpeg_decoder_switch_buffer(int cam_num) {
	if (!lg_send_frame_arg[cam_num]) {
		return;
	}
	int res = mrevent_wait(&lg_send_frame_arg[cam_num]->buffer_ready,
			1000 * 1000);
	if (res != 0) {
		mrevent_trigger(&lg_send_frame_arg[cam_num]->buffer_ready);
	}
}
