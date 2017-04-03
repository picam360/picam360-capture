#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <wchar.h>

#include "driver_agent.h"
#include "kokuyoseki.h"
#include "rtp.h"

#include <mat4/identity.h>
#include <mat4/rotateY.h>
#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/invert.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PLUGIN_NAME "driver_agent"

#define PT_STATUS 100
#define PT_CMD 101
#define PT_CAM_BASE 110

static PLUGIN_HOST_T *lg_plugin_host = NULL;
static int lg_status_fd = -1;
static int lg_cam0_fd = -1;
static int lg_cam1_fd = -1;
static bool lg_recording = false;

static float lg_bandwidth = 0.0;

static void release(void *user_data) {
	free(user_data);
}

static int picam360_driver_xmp(char *buff, int buff_len, float light0_value,
		float light1_value, float motor0_value, float motor1_value,
		float motor2_value, float motor3_value) {
	int xmp_len = 0;

	xmp_len = 0;
	buff[xmp_len++] = 0xFF;
	buff[xmp_len++] = 0xE1;
	buff[xmp_len++] = 0; // size MSB
	buff[xmp_len++] = 0; // size LSB
	xmp_len += sprintf(buff + xmp_len, "http://ns.adobe.com/xap/1.0/");
	buff[xmp_len++] = '\0';
	xmp_len += sprintf(buff + xmp_len, "<?xpacket begin=\"﻿");
	buff[xmp_len++] = 0xEF;
	buff[xmp_len++] = 0xBB;
	buff[xmp_len++] = 0xBF;
	xmp_len += sprintf(buff + xmp_len, "\" id=\"W5M0MpCehiHzreSzNTczkc9d\"?>");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<x:xmpmeta xmlns:x=\"adobe:ns:meta/\" x:xmptk=\"picam360-capture rev1\">");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">");
	xmp_len += sprintf(buff + xmp_len, "<rdf:Description rdf:about=\"\">");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<picam360_driver"
							" light0_value=\"%f\" light1_value=\"%f\""
							" motor0_value=\"%f\" motor1_value=\"%f\" motor2_value=\"%f\" motor3_value=\"%f\""
							" />", light0_value, light1_value, motor0_value,
					motor1_value, motor2_value, motor3_value);
	xmp_len += sprintf(buff + xmp_len, "</rdf:Description>");
	xmp_len += sprintf(buff + xmp_len, "</rdf:RDF>");
	xmp_len += sprintf(buff + xmp_len, "</x:xmpmeta>");
	xmp_len += sprintf(buff + xmp_len, "<?xpacket end=\"w\"?>");
	buff[xmp_len++] = '\0';
	buff[2] = ((xmp_len - 2) >> 8) & 0xFF; // size MSB
	buff[3] = (xmp_len - 2) & 0xFF; // size LSB

	return xmp_len;
}

#define LIGHT_NUM 2
#define MOTOR_NUM 4
static int lg_light_value[LIGHT_NUM] = { 0, 0 };
static int lg_motor_value[MOTOR_NUM] = { 0, 0, 0, 0 };
static float lg_light_strength = 0; //0 to 100
static float lg_thrust = 0; //-100 to 100
static float lg_brake_ps = 5; // percent
static bool lg_lowlevel_control = false;
static float lg_camera_quatanion[4] = { 0, 0, 0, 1 };
static float lg_camera_compass[4] = { 0, 0, 0, 1 };
static float lg_camera_north = 0;
static float lg_camera_north_count = 0;
static float lg_target_quatanion[4] = { 0, 0, 0, 1 };

#define MAX_DELAY_COUNT 256
static int lg_delay = 0;
static int lg_delay_cur = 0;
static float lg_camera_quatanion_queue[MAX_DELAY_COUNT][4] = { };

static bool lg_pid_enabled = false;
static float lg_p_gain = 1.0;
static float lg_i_gain = 1.0;
static float lg_d_gain = 1.0;
static float lg_pid_value[3] = { }; //x, z, delta yaw
static float lg_delta_pid_target[3][3] = { }; //x, z, delta yaw
static struct timeval lg_delta_pid_time[3] = { };

//kokuyoseki
static struct timeval lg_last_kokuyoseki_time = { };
static int lg_last_button = -1;
static int lg_func = -1;

#define NUM_OF_CAM 2
static float lg_fps[NUM_OF_CAM] = { };
static int lg_frameskip[NUM_OF_CAM] = { };

static void parse_xml(char *xml) {
	char *q_str = NULL;
	q_str = strstr(xml, "<quaternion");
	if (q_str) {
		int cur = (lg_delay_cur) % MAX_DELAY_COUNT;
		int delay_cur = (lg_delay_cur - lg_delay + MAX_DELAY_COUNT)
				% MAX_DELAY_COUNT;
		float quatanion[4];
		sscanf(q_str, "<quaternion w=\"%f\" x=\"%f\" y=\"%f\" z=\"%f\" />",
				&quatanion[0], &quatanion[1], &quatanion[2], &quatanion[3]);
		//convert from mpu coodinate to opengl coodinate
		lg_camera_quatanion_queue[cur][0] = quatanion[1]; //x
		lg_camera_quatanion_queue[cur][1] = quatanion[3]; //y : swap y and z
		lg_camera_quatanion_queue[cur][2] = -quatanion[2]; //z : swap y and z
		lg_camera_quatanion_queue[cur][3] = quatanion[0]; //w
		memcpy(lg_camera_quatanion, lg_camera_quatanion_queue[delay_cur],
				sizeof(float) * 4);
		lg_plugin_host->set_camera_quatanion(lg_camera_quatanion);

		lg_delay_cur++;
	}
	q_str = strstr(xml, "<compass");
	if (q_str) {
		float compass[3];
		sscanf(q_str, "<compass x=\"%f\" y=\"%f\" z=\"%f\" />", &compass[0],
				&compass[1], &compass[2]);
		//convert from mpu coodinate to opengl coodinate
		lg_camera_compass[0] = compass[1];
		lg_camera_compass[1] = -compass[0];
		lg_camera_compass[2] = -compass[2];
		lg_camera_compass[3] = 1.0;

		lg_plugin_host->set_camera_compass(lg_camera_compass);

		{ //north
			float north = 0;

			float matrix[16];
			mat4_fromQuat(matrix, lg_camera_quatanion);
			mat4_invert(matrix, matrix);

			float compass_mat[16] = { };
			memcpy(compass_mat, lg_camera_compass, sizeof(float) * 4);

			mat4_transpose(compass_mat, compass_mat);
			mat4_multiply(compass_mat, compass_mat, matrix);
			mat4_transpose(compass_mat, compass_mat);

			north = -atan2(compass_mat[2], compass_mat[0]) * 180 / M_PI;

			lg_camera_north = (lg_camera_north * lg_camera_north_count + north)
					/ (lg_camera_north_count + 1);
			lg_camera_north_count++;
			if (lg_camera_north_count > 1000) {
				lg_camera_north_count = 1000;
			}
			lg_plugin_host->set_camera_north(lg_camera_north);
		}
	}
	q_str = strstr(xml, "<temperature");
	if (q_str) {
		float temperature;
		sscanf(q_str, "<temperature v=\"%f\" />", &temperature);

		lg_plugin_host->set_camera_temperature(temperature);
	}
	q_str = strstr(xml, "<bandwidth");
	if (q_str) {
		float bandwidth;
		sscanf(q_str, "<bandwidth v=\"%f\" />", &bandwidth);
		lg_bandwidth = bandwidth;
	}
	{
		int offset = 0;
		do {
			q_str = strstr(xml + offset, "<video_info");
			offset = (unsigned long) q_str - (unsigned long) xml + 1;
			if (q_str) {
				int id;
				float fps;
				int frameskip;
				sscanf(q_str,
						"<video_info id=\"%d\" fps=\"%f\" frameskip=\"%d\" />",
						&id, &fps, &frameskip);
				if (id >= 0 && id < NUM_OF_CAM) {
					lg_fps[id] = fps;
					lg_frameskip[id] = frameskip;
				}
			}
		} while (q_str);
	}
}

static void *recieve_thread_func(void* arg) {
	int buff_size = RTP_MAXPAYLOADSIZE;
	unsigned char *buff = malloc(buff_size);
	int data_len = 0;
	int marker = 0;
	int camd_fd = -1;
	bool xmp = false;
	int buff_xmp_size = 4096;
	char *buff_xmp = malloc(buff_xmp_size);
	int xmp_len = 0;
	int xmp_idx = 0;

	while (1) {
		bool reset = false;
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
					if (xmp_len > buff_xmp_size) {
						free(buff_xmp);
						buff_xmp_size = xmp_len;
						buff_xmp = malloc(buff_xmp_size);
					}
					buff_xmp[0] = (xmp_len >> 8) & 0xFF;
					buff_xmp[1] = (xmp_len) & 0xFF;
				} else {
					buff_xmp[xmp_idx] = buff[i];
				}
				xmp_idx++;
				if (xmp_idx >= xmp_len) {
					char *xml = buff_xmp + strlen(buff_xmp) + 1;

					parse_xml(xml);
					xmp = false;
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

	free(buff);
	free(buff_xmp);

	return NULL;
}

float sub_angle(float a, float b) {
	float v = a - b;
	v -= floor(v / 360) * 360;
	if (v > 180.0) {
		v -= 360.0;
	}
	return v;
}

void *transmit_thread_func(void* arg) {
	int xmp_len = 0;
	int buff_size = 4096;
	char buff[buff_size];
	static struct timeval last_time = { };
	gettimeofday(&last_time, NULL);
	while (1) {
		struct timeval time = { };
		gettimeofday(&time, NULL);
		struct timeval diff;
		timersub(&time, &last_time, &diff);
		float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
		//cal
		if (!lg_lowlevel_control) {
			//trancate min max
			lg_light_strength = MIN(MAX(lg_light_strength, 0), 100);
			lg_light_value[0] = lg_light_strength;
			lg_light_value[1] = lg_light_strength;

			//trancate min max
			lg_thrust = MIN(MAX(lg_thrust, -100), 100);
			//brake
			lg_thrust *= exp(log(1.0 - lg_brake_ps / 100) * diff_sec);

			if (lg_pid_enabled) {
				//(RcRt-1Rc-1)*(Rc)*vtg, target coordinate will be converted into camera coordinate
				float vtg[16] = { 0, -1, 0, 1 }; // looking at ground
				float unif_matrix[16];
				float camera_matrix[16];
				float target_matrix[16];
				float north_matrix[16];
				mat4_identity(unif_matrix);
				mat4_identity(camera_matrix);
				mat4_identity(target_matrix);
				mat4_identity(north_matrix);
				mat4_fromQuat(camera_matrix, lg_camera_quatanion);
				mat4_fromQuat(target_matrix, lg_target_quatanion);
				mat4_invert(target_matrix, target_matrix);
				// Rn
				{
					float north_diff = lg_plugin_host->get_view_north()
							- lg_plugin_host->get_camera_north();
					mat4_rotateY(north_matrix, north_matrix,
							north_diff * M_PI / 180);
				}
				mat4_multiply(unif_matrix, unif_matrix, target_matrix); // Rt-1
				mat4_multiply(unif_matrix, unif_matrix, north_matrix); // RnRt-1Rw
				mat4_multiply(unif_matrix, unif_matrix, camera_matrix); // RcRt-1

				mat4_transpose(vtg, vtg);
				mat4_multiply(vtg, vtg, unif_matrix);
				mat4_transpose(vtg, vtg);

				float xz = sqrt(vtg[0] * vtg[0] + vtg[2] * vtg[2]);
				float yaw = -atan2(vtg[2], vtg[0]) * 180 / M_PI;
				float pitch = atan2(xz, vtg[1]) * 180 / M_PI;

				static float last_yaw = 0;
				lg_delta_pid_time[0] = time;
				lg_delta_pid_target[0][0] = cos(yaw * M_PI / 180)
						* (pitch / 180); // x [-1:1]
				lg_delta_pid_target[1][0] = sin(yaw * M_PI / 180)
						* (pitch / 180); // z [-1:1]
				lg_delta_pid_target[2][0] = sub_angle(yaw, last_yaw) / 180; // delta yaw [-1:1]

				timersub(&lg_delta_pid_time[0], &lg_delta_pid_time[1], &diff);
				diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
				diff_sec = MAX(MIN(diff_sec, 1.0), 0.001);

				for (int k = 0; k < 3; k++) {
					float p_value = lg_p_gain
							* (lg_delta_pid_target[k][0]
									- lg_delta_pid_target[k][1]);
					float i_value = lg_i_gain * lg_delta_pid_target[k][0]
							* diff_sec;
					float d_value = lg_d_gain
							* (lg_delta_pid_target[k][0]
									- 2 * lg_delta_pid_target[k][1]
									+ lg_delta_pid_target[k][2]) / diff_sec;
					float delta_value = p_value + i_value + d_value;
					lg_pid_value[k] += delta_value;
					lg_pid_value[k] = MIN(MAX(lg_pid_value[k], -2500), 2500);
				}

				//increment
				for (int j = 3 - 1; j >= 1; j--) {
					for (int k = 0; k < 3; k++) {
						lg_delta_pid_target[k][j] =
								lg_delta_pid_target[k][j - 1];
					}
					lg_delta_pid_time[j] = lg_delta_pid_time[j - 1];
				}
				last_yaw = yaw;

				// 0 - 1
				// |   |
				// 3 - 2
				float motor_pid_gain[3][MOTOR_NUM] = { //
						//
								{ 1, -1, -1, 1 },				// x
								{ -1, -1, 1, 1 },				// z
								{ -1, 1, -1, 1 }			// delta yaw
						};
				float motor_pid_value[MOTOR_NUM] = { };
				for (int k = 0; k < 3; k++) {
					for (int i = 0; i < MOTOR_NUM; i++) {
						float power_calib = (lg_pid_value[k] > 0 ? 1 : -1)
								* sqrt(abs(lg_pid_value[k]));
						motor_pid_value[i] += power_calib
								* motor_pid_gain[k][i];
					}
				}
				for (int i = 0; i < MOTOR_NUM; i++) {
					float value = lg_thrust + motor_pid_value[i];
					float diff = value - lg_motor_value[i];
					int max_diff = 10;
					if (abs(diff) > max_diff) {
						diff = (diff > 0) ? max_diff : -max_diff;
					}
					value = lg_motor_value[i] + diff;
					if (value * lg_motor_value[i] < 0) {
						value = 0;
					}
					lg_motor_value[i] = value;
				}
				if (1) {
					printf("yaw=%f,\tpitch=%f\tpid_value=%f\tdelta_value=%f",
							yaw, pitch, lg_pid_value[0],
							lg_delta_pid_target[0][0]);
					for (int i = 0; i < MOTOR_NUM; i++) {
						printf(", m%d=%d", i, lg_motor_value[i]);
					}
					printf("\n");
				} // end of pid control
			} else {
				for (int i = 0; i < MOTOR_NUM; i++) {
					float value = lg_thrust;
					float diff = value - lg_motor_value[i];
					int max_diff = 10;
					if (abs(diff) > max_diff) {
						diff = (diff > 0) ? max_diff : -max_diff;
					}
					value = lg_motor_value[i] + diff;
					if (value * lg_motor_value[i] < 0) {
						value = 0;
					}
					lg_motor_value[i] = MIN(MAX(value, -100), 100);
				}
			}
		} // end of !low_motor_control
		  //kokuyoseki func
		if (lg_last_button == BLACKOUT_BUTTON && lg_func != -1) {
			timersub(&time, &lg_last_kokuyoseki_time, &diff);
			diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
			if (diff_sec > 0.5) {
				printf("func %d: ", lg_func);
				switch (lg_func) {
				case 1:
					lg_light_strength = 0;
					printf("light off\n");
					break;
				case 2:
					lg_thrust = 0;
					printf("thrust off\n");
					break;
				case 3:
					lg_thrust = 0;
					for (int k = 0; k < 3; k++) {
						lg_pid_value[k] = 0;
					}
					if (lg_pid_enabled) {
						lg_pid_enabled = false;
						printf("pid off\n");
					} else {
						lg_pid_enabled = true;
						printf("pid on\n");
					}
					break;
				case 4:
					if (lg_recording) {
						rtp_stop_recording();
						printf("stop recording\n");
						lg_recording = false;
					} else {
						struct tm *tmptr = NULL;
						tmptr = localtime(&time.tv_sec);

						char filename[256];
						sprintf(filename,
								"/media/usbdisk/%04d-%02d-%02d_%02d-%02d-%02d.rtp",
								tmptr->tm_year + 1900, tmptr->tm_mon + 1,
								tmptr->tm_mday, tmptr->tm_hour, tmptr->tm_min,
								tmptr->tm_sec);
						rtp_start_recording(filename);
						printf("start recording %s\n", filename);
						lg_recording = true;
					}
					break;
				}
				if (lg_func > 10) {
					exit(0);
				}
				lg_func = -1;
			}
		}

		xmp_len = picam360_driver_xmp(buff, sizeof(buff), lg_light_value[0],
				lg_light_value[1], lg_motor_value[0], lg_motor_value[1],
				lg_motor_value[2], lg_motor_value[3]);

		rtp_sendpacket((unsigned char*) buff, xmp_len, PT_CMD);

		last_time = time;
		usleep(100 * 1000); //less than 10Hz
	} // end of while
}

static void loading_callback(int ret) {
	printf("end of loading\n");
}

static void command_handler(void *user_data, char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, PLUGIN_NAME ".set_light_value", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_light_value[0] = value;
			lg_light_value[1] = value;
			printf("set_light_value : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_motor_value", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int id = 0;
			float value = 0;
			sscanf(param, "%d=%f", &id, &value);
			if (id < MOTOR_NUM) {
				lg_motor_value[id] = value;
			}
			printf("set_motor_value : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_delay", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value = 0;
			sscanf(param, "%f", &value);
			lg_delay = MAX(MIN((int) value,MAX_DELAY_COUNT), 0);
			printf("set_delay : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".start_recording", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			rtp_start_recording(param);
			printf("start_recording : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".stop_recording", sizeof(buff)) == 0) {
		rtp_stop_recording();
		printf("stop_recording : completed\n");
	} else if (strncmp(cmd, PLUGIN_NAME ".start_loading", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			rtp_start_loading(param, (RTP_LOADING_CALLBACK) loading_callback);
			printf("start_loading : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".stop_loading", sizeof(buff)) == 0) {
		rtp_stop_loading();
		printf("stop_loading : completed\n");
	} else {
		printf(":unknown command : %s\n", buff);
	}
}

static void kokuyoseki_callback(struct timeval time, int button, int value) {
	if (value == 1) {
		return;
	}
	struct timeval diff;
	timersub(&time, &lg_last_kokuyoseki_time, &diff);
	float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
	switch (button) {
	case NEXT_BUTTON:
		lg_thrust += 1;
		printf("thrust %f\n", lg_thrust);
		break;
	case BACK_BUTTON:
		lg_thrust -= 1;
		printf("thrust %f\n", lg_thrust);
		break;
	case NEXT_BUTTON_LONG:
		if (diff_sec < 0.25)
			return;
		lg_light_strength += 1;
		printf("light %f\n", lg_light_strength);
		break;
	case BACK_BUTTON_LONG:
		if (diff_sec < 0.25)
			return;
		lg_light_strength -= 1;
		printf("light %f\n", lg_light_strength);
		break;
	case BLACKOUT_BUTTON:
		lg_func++;
		break;
	}
	{
		float *quat;
		switch (button) {
		case NEXT_BUTTON:
		case BACK_BUTTON:
			quat = lg_plugin_host->get_view_quatanion();
			if (quat) {
				memcpy(lg_target_quatanion, quat, sizeof(lg_target_quatanion));
			}
		}
	}
	lg_last_kokuyoseki_time = time;
	lg_last_button = button;
}

static void init_options(void *user_data, json_t *options) {
	lg_p_gain = json_number_value(
			json_object_get(options, PLUGIN_NAME ".p_gain"));
	lg_i_gain = json_number_value(
			json_object_get(options, PLUGIN_NAME ".i_gain"));
	lg_d_gain = json_number_value(
			json_object_get(options, PLUGIN_NAME ".d_gain"));
	lg_delay = (int) json_number_value(
			json_object_get(options, PLUGIN_NAME ".delay"));
}

static void save_options(void *user_data, json_t *options) {
	json_object_set_new(options, PLUGIN_NAME ".p_gain", json_real(lg_p_gain));
	json_object_set_new(options, PLUGIN_NAME ".i_gain", json_real(lg_i_gain));
	json_object_set_new(options, PLUGIN_NAME ".d_gain", json_real(lg_d_gain));
	json_object_set_new(options, PLUGIN_NAME ".delay",
			json_real((float) lg_delay));
}

static int rtp_callback(char *data, int data_len, int pt) {
	if (pt == PT_STATUS) {
		int fd = -1;
		if (lg_status_fd < 0) {
			lg_status_fd = open("status", O_WRONLY | O_NONBLOCK);
		}
		fd = lg_status_fd;
		if (fd < 0) {
			return -1;
		}
		write(fd, data, data_len);
	} else if (pt == PT_CAM_BASE + 0) {
		lg_plugin_host->decode_video(0, (unsigned char*) data, data_len);
	} else if (pt == PT_CAM_BASE + 1) {
		lg_plugin_host->decode_video(1, (unsigned char*) data, data_len);
	}
	return 0;
}

static bool is_init = false;
static void init() {
	if (is_init) {
		return;
	}
	is_init = true;

	init_rtp(9002, "192.168.4.1", 9004);
	rtp_set_callback((RTP_CALLBACK) rtp_callback);

	set_kokuyoseki_callback((KOKUYOSEKI_CALLBACK) kokuyoseki_callback);
	open_kokuyoseki();

	pthread_t transmit_thread;
	pthread_create(&transmit_thread, NULL, transmit_thread_func, (void*) NULL);

	pthread_t recieve_thread;
	pthread_create(&recieve_thread, NULL, recieve_thread_func, (void*) NULL);
}

#define MAX_INFO_LEN 1024
static wchar_t lg_info[MAX_INFO_LEN];
static wchar_t *get_info(void *user_data) {
	int cur = 0;
	cur += swprintf(lg_info, MAX_INFO_LEN,
			L"rx %.1f Mbps, fps %.1f:%.1f skip %d:%d", lg_bandwidth, lg_fps[0],
			lg_fps[1], lg_frameskip[0], lg_frameskip[1]);
	if (rtp_is_recording(NULL)) {
		char *path;
		rtp_is_recording(&path);
		cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur, L", to %hs", path);
	}
	if (rtp_is_loading(NULL)) {
		char *path;
		rtp_is_loading(&path);
		cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur, L", from %hs", path);
	}
	return lg_info;
}

void create_driver_agent(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	init();
	lg_plugin_host = plugin_host;

	PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(PLUGIN_T));
	strcpy(plugin->name, PLUGIN_NAME);
	plugin->release = release;
	plugin->command_handler = command_handler;
	plugin->init_options = init_options;
	plugin->save_options = save_options;
	plugin->get_info = get_info;
	plugin->user_data = plugin;

	*_plugin = plugin;
}
