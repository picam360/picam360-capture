/*
 Copyright (c) 2012, Broadcom Europe Ltd
 Copyright (c) 2012, OtherCrashOverride
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Video decode demo using OpenMAX IL though the ilcient helper library
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "bcm_host.h"
#include "ilclient.h"
#include "picam360_capture.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static OMX_BUFFERHEADERTYPE* eglBuffer[2] = { };
static COMPONENT_T* egl_render[2] = { };

static void* eglImage[2] = { };

static void my_fill_buffer_done(void* data, COMPONENT_T* comp) {
	int index = (int) data;

	if (OMX_FillThisBuffer(ilclient_get_handle(egl_render[index]),
			eglBuffer[index]) != OMX_ErrorNone) {
		printf("test  OMX_FillThisBuffer failed in callback\n");
		exit(1);
	}
}

static pthread_mutex_t image_mlock = PTHREAD_MUTEX_INITIALIZER;
typedef struct _IMAGE_DATA {
	int refcount;
	int image_size;
	unsigned char *image_buff;
} IMAGE_DATA;

static IMAGE_DATA *create_image(int image_buff_size) {
	IMAGE_DATA *image_data = malloc(sizeof(IMAGE_DATA) + image_buff_size);
	memset(image_data, 0, sizeof(IMAGE_DATA));
	image_data->refcount = 1;
	image_data->image_buff = (unsigned char*) image_data + sizeof(IMAGE_DATA);
	image_data->image_size = image_buff_size;
	return image_data;
}

static int addref_image(IMAGE_DATA *image_data) {
	if (image_data == NULL)
		return 0;

	int ret;
	pthread_mutex_lock(&image_mlock);
	if (image_data->refcount == 0) {
		ret = 0;
	} else {
		image_data->refcount++;
		ret = image_data->refcount;
	}
	pthread_mutex_unlock(&image_mlock);
	return ret;
}

static int release_image(IMAGE_DATA *image_data) {
	if (image_data == NULL)
		return 0;

	int ret;
	pthread_mutex_lock(&image_mlock);
	if (--image_data->refcount == 0) {
		free(image_data);
		ret = 0;
	} else {
		ret = image_data->refcount;
	}
	pthread_mutex_unlock(&image_mlock);
	return ret;
}

typedef struct _IMAGE_RECEIVER_DATA {
	PICAM360CAPTURE_T *state;
	int index;
	pthread_mutex_t *mlock_p;
	IMAGE_DATA *image_data;
} IMAGE_RECEIVER_DATA;

void *image_dumper(void* arg) {
	IMAGE_RECEIVER_DATA *data = (IMAGE_RECEIVER_DATA*) arg;
	IMAGE_DATA *image_data = NULL;
	int descriptor = -1;
	while (1) {

		//wait untill image arived
		if (data->image_data == NULL || data->image_data == image_data) {
			usleep(1000);
			continue;
		}

		pthread_mutex_lock(data->mlock_p);
		if (image_data != NULL) { // release memory
			release_image(image_data);
		}
		image_data = data->image_data;
		addref_image(image_data);
		pthread_mutex_unlock(data->mlock_p);

		if (descriptor >= 0) {
			if (!data->state->output_raw) { //end
				close(descriptor);
				descriptor = -1;
				continue;
			} else { // write
				write(descriptor, image_data->image_buff,
						image_data->image_size);
			}
		} else if (data->state->output_raw) { // start
			char buff[256];
			sprintf(buff, data->state->output_raw_filepath, data->index);
			descriptor = open(buff, O_WRONLY | O_CREAT, 0666);
			if (descriptor == -1) {
				printf("failed to open %s\n", buff);
				data->state->output_raw = false;
				continue;
			}
		} else {
			usleep(1000);
		}
	}

	return NULL;
}

void *image_receiver(void* arg) {
	IMAGE_RECEIVER_DATA *data = (IMAGE_RECEIVER_DATA*) arg;
	int buff_size = 4096;
	unsigned char *buff = malloc(buff_size);
	unsigned char *buff_trash = malloc(buff_size);
	IMAGE_DATA *image_data = NULL;
	int image_buff_size = 0;
	int image_buff_cur = 0;
	int image_start = -1;
	int data_len = 0;
	int data_len_total = 0;
	int marker = 0;
	int soicount = 0;
	int camd_fd = -1;
	int file_fd = -1;

	while (1) {
		bool reset = false;
		if (data->state->input_mode == INPUT_MODE_CAM) {
			if (cam_fd < 0) {
				char buff[256];
				sprintf(buff, "cam%d", data->index);
				camd_fd = open(buff, O_RDONLY);
				if (camd_fd == -1) {
					printf("failed to open %s\n", buff);
					exit(-1);
				}
				printf("%s ready\n", buff);
			}
			data_len = read(camd_fd, buff, buff_size);
			if (data_len == 0) {
				printf("camera input invalid\n");
				break;
			}
		} else if (camd_fd >= 0) {
			read(camd_fd, buff_trash, buff_size);
		}
		if (file_fd >= 0) {
			if (data->state->input_mode != INPUT_MODE_FILE) { // end
				close(file_fd);
				file_fd = -1;
				data->state->input_mode = INPUT_MODE_CAM;
				reset = true;
			} else { //read
				if (data->state->frame_sync) {
					int res = mrevent_wait(
							&data->state->request_frame_event[data->index],
							1000); //wait 1msec
					if (res != 0) {
						continue;
					}
				}

				if (data->state->input_file_cur
						< data->state->input_file_size) {
					data_len = read(file_fd, buff, buff_size);
					data->state->input_file_cur += data_len;
				}
			}
		} else if (data->state->input_mode == INPUT_MODE_FILE) { //start
			char buff[256];
			sprintf(buff, data->state->input_filepath, data->index);
			file_fd = open(buff, O_RDONLY);
			if (file_fd == -1) {
				printf("failed to open %s\n", buff);
				data->state->input_mode = INPUT_MODE_CAM;
			}
			struct stat st;
			stat(buff, &st);
			data->state->input_file_size = st.st_size;
			data->state->input_file_cur = 0;

			printf("open %s : %ldB\n", buff, (long int) st.st_size);

			reset = true;
		}
		if (reset) {
			image_buff_cur = 0;
			image_start = -1;
			data_len_total = 0;
			marker = 0;
			soicount = 0;
			continue;
		}
		for (int i = 0; i < data_len; i++) {
			if (marker) {
				marker = 0;
				if (buff[i] == 0xd8) { //SOI
					if (soicount == 0) {
						image_start = data_len_total + (i - 1);
					}
					soicount++;
				}
				if (buff[i] == 0xd9 && image_start >= 0) { //EOI
					soicount--;
					if (soicount == 0) {
						int image_size = (data_len_total - image_start)
								+ (i + 1);

						if (image_data == NULL) { //just allocate image buffer
							image_buff_size = image_size * 2;
						} else if (image_size > image_buff_size) { //exceed buffer size
							free(image_data);
							image_data = NULL;
							image_buff_size = image_size * 2;
						} else {
							if (image_start > data_len_total) { //soi
								memcpy(image_data->image_buff,
										buff + (image_start - data_len_total),
										(i + 1)
												- (image_start - data_len_total));
							} else {
								memcpy(image_data->image_buff + image_buff_cur,
										buff, i + 1);
							}
							image_data->image_size = image_size;
							pthread_mutex_lock(data->mlock_p);
							if (data->image_data != NULL) {
								release_image(data->image_data);
							}
							data->image_data = image_data;
							pthread_mutex_unlock(data->mlock_p);

							mrevent_reset(
									&data->state->request_frame_event[data->index]);
							mrevent_trigger(
									&data->state->arrived_frame_event[data->index]);
						}
						image_buff_cur = 0;
						image_data = create_image(image_buff_size);
						image_start = -1;
					}
				}
			} else if (buff[i] == 0xff) {
				marker = 1;
			}
		}
		if (image_data != NULL && image_start >= 0) {
			if (image_buff_cur + data_len > image_buff_size) { //exceed buffer size
				free(image_data);
				image_data = NULL;
			} else if (image_start > data_len_total) { //soi
				memcpy(image_data->image_buff,
						buff + (image_start - data_len_total),
						data_len - (image_start - data_len_total));
				image_buff_cur = data_len - (image_start - data_len_total);
			} else {
				memcpy(image_data->image_buff + image_buff_cur, buff, data_len);
				image_buff_cur += data_len;
			}
		}
		data_len_total += data_len;
	}

	return NULL;
}

// Modified function prototype to work with pthreads
void *video_mjpeg_decode(void* arg) {
	int index;
	PICAM360CAPTURE_T *state;

	index = (int) ((void**) arg)[0];
	eglImage[index] = ((void**) arg)[1];
	state = (PICAM360CAPTURE_T *) ((void**) arg)[2];

	if (eglImage[index] == 0) {
		printf("eglImage is null.\n");
		exit(1);
	}

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

	// create egl_render
	if (status == 0
			&& ilclient_create_component(client, &egl_render[index],
					"egl_render",
					ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_OUTPUT_BUFFERS)
					!= 0)
		status = -14;
	list[1] = egl_render[index];

	set_tunnel(tunnel, video_decode, 131, egl_render[index], 220);

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

		pthread_mutex_t mlock;
		pthread_mutex_init(&mlock, NULL);
		IMAGE_RECEIVER_DATA data = { };
		data.state = state;
		data.index = index;
		data.mlock_p = &mlock;

		pthread_t image_receiver_thread;
		pthread_create(&image_receiver_thread, NULL, image_receiver,
				(void*) &data);

		pthread_t image_dumper_thread;
		pthread_create(&image_dumper_thread, NULL, image_dumper, (void*) &data);

		IMAGE_DATA *image_data = NULL;
		while (1) {
			int image_cur = 0;

			//wait untill image arived
			if (data.image_data == NULL || data.image_data == image_data) {
				usleep(1000);
				continue;
			}

			pthread_mutex_lock(&mlock);
			if (image_data != NULL) { // release memory
				release_image(image_data);
			}
			image_data = data.image_data;
			addref_image(image_data);
			pthread_mutex_unlock(&mlock);

			while (image_cur < image_data->image_size) {
				buf = ilclient_get_input_buffer(video_decode, 130, 1);

				data_len = MIN(buf->nAllocLen,
						image_data->image_size - image_cur);
				memcpy(buf->pBuffer, image_data->image_buff + image_cur,
						data_len);
				image_cur += data_len;

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

					// Set egl_render to idle
					ilclient_change_component_state(egl_render[index],
							OMX_StateIdle);

					// Enable the output port and tell egl_render to use the texture as a buffer
					//ilclient_enable_port(egl_render, 221); THIS BLOCKS SO CAN'T BE USED
					if (OMX_SendCommand(ILC_GET_HANDLE(egl_render[index]),
							OMX_CommandPortEnable, 221, NULL)
							!= OMX_ErrorNone) {
						printf("OMX_CommandPortEnable failed.\n");
						exit(1);
					}

					if (OMX_UseEGLImage(ILC_GET_HANDLE(egl_render[index]),
							&eglBuffer[index], 221, NULL, eglImage[index])
							!= OMX_ErrorNone) {
						printf("OMX_UseEGLImage failed.\n");
						exit(1);
					}

					// Set egl_render to executing
					ilclient_change_component_state(egl_render[index],
							OMX_StateExecuting);

					// Request egl_render to write data to the texture buffer
					if (OMX_FillThisBuffer(ILC_GET_HANDLE(egl_render[index]),
							eglBuffer[index]) != OMX_ErrorNone) {
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

				if (image_cur >= image_data->image_size) {
					buf->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;
				}

				if (OMX_EmptyThisBuffer(ILC_GET_HANDLE(video_decode), buf)
						!= OMX_ErrorNone) {
					status = -6;
					break;
				}
			}
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

