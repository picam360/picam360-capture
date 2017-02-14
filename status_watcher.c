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

#include "status_watcher.h"
#include "picam360_capture.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static ATTITUDE_CALLBACK lg_attitude_callback = NULL;

void set_attitude_callback(ATTITUDE_CALLBACK callback) {
	lg_attitude_callback = callback;
}

static void *image_receiver(void* arg) {
	PICAM360CAPTURE_T *state = (PICAM360CAPTURE_T *) arg;

	int buff_size = 4096;
	unsigned char *buff = malloc(buff_size);
	unsigned char *buff_trash = malloc(buff_size);
	int data_len = 0;
	int marker = 0;
	int camd_fd = -1;
	int file_fd = -1;
	bool xmp = false;
	char *buff_xmp = NULL;
	int xmp_len = 0;
	int xmp_idx = 0;

	while (1) {
		bool reset = false;
		if (state->input_mode == INPUT_MODE_CAM) {
			if (camd_fd < 0) {
				char buff[256];
				sprintf(buff, "status");
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
			if (state->input_mode != INPUT_MODE_FILE) { // end
				close(file_fd);
				file_fd = -1;
				state->input_mode = INPUT_MODE_CAM;
				reset = true;
			} else { //read
				if (state->input_file_cur
						< state->input_file_size) {
					data_len = read(file_fd, buff, buff_size);
					state->input_file_cur += data_len;
				}
			}
		} else if (state->input_mode == INPUT_MODE_FILE) { //start
			char buff[256];
			sprintf(buff, state->input_filepath, index);
			file_fd = open(buff, O_RDONLY);
			if (file_fd == -1) {
				printf("failed to open %s\n", buff);
				state->input_mode = INPUT_MODE_CAM;
			}
			struct stat st;
			stat(buff, &st);
			state->input_file_size = st.st_size;
			state->input_file_cur = 0;

			printf("open %s : %ldB\n", buff, (long int) st.st_size);

			reset = true;
		}
		if (reset) {
			marker = 0;
			continue;
		}
		for (int i = 0; i < data_len; i++) {
			if (xmp) {
				if (xmp_idx == 0) {
					xmp_len = ((unsigned char*) buff)[i] << 8;
				} else if (xmp_idx == 1) {
					xmp_len += ((unsigned char*) buff)[i];
					buff_xmp = malloc(xmp_len);
					buff_xmp[0] = (xmp_len >> 8) & 0xFF;
					buff_xmp[1] = (xmp_len) & 0xFF;
				} else {
					buff_xmp[xmp_idx] = buff[i];
				}
				xmp_idx++;
				if (xmp_idx >= xmp_len) {
					char *xml = buff_xmp + strlen(buff_xmp) + 1;

					char *q_str = strstr(xml, "<quaternion");
					if (q_str) {
						float quat[4];
						float quatanion[4];
						sscanf(q_str,
								"<quaternion w=\"%f\" x=\"%f\" y=\"%f\" z=\"%f\" />",
								&quatanion[0], &quatanion[1], &quatanion[2], &quatanion[3]);
						//convert from mpu coodinate to opengl coodinate
						quat[0] = quatanion[1];//x
						quat[1] = quatanion[3];//y : swap y and z
						quat[2] = -quatanion[2];//z : swap y and z
						quat[3] = quatanion[0];//w

						if (lg_attitude_callback) {
							lg_attitude_callback(quat);
						}
					}

					xmp = false;
					free(buff_xmp);
					buff_xmp = NULL;
				}
			}
			if (marker) {
				marker = 0;
				if (buff[i] == 0xE1) { //APP1
					xmp = true;
					xmp_len = 0;
					xmp_idx = 0;
				}
			} else if (buff[i] == 0xFF) {
				marker = 1;
			}
		}
	}

	return NULL;
}

// Modified function prototype to work with pthreads
void *status_watch(void* arg) {
	PICAM360CAPTURE_T *state = (PICAM360CAPTURE_T *) arg;

	printf("milestone\n");

	pthread_t image_receiver_thread;
	pthread_create(&image_receiver_thread, NULL, image_receiver, (void*) state);

	return (void*) NULL;
}

