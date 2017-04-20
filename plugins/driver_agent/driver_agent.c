#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <wchar.h>
#include <limits.h>
#include <dirent.h>
#include <pthread.h>

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
#define PACKET_FOLDER_PATH "/media/usbdisk/packet"
#define STILL_FOLDER_PATH "/media/usbdisk/still"
#define VIDEO_FOLDER_PATH "/media/usbdisk/video"

#define PT_STATUS 100
#define PT_CMD 101
#define PT_CAM_BASE 110

enum UI_MODE {
	UI_MODE_DEFAULT, UI_MODE_LIGHT, UI_MODE_FOV,
} lg_ui_mode = UI_MODE_DEFAULT;
enum SYSTEM_CMD {
	SYSTEM_CMD_NONE, SYSTEM_CMD_SHUTDOWN,
};

static PLUGIN_HOST_T *lg_plugin_host = NULL;

#define MAX_DELAY_COUNT 256
static float lg_video_delay = 0.0;
static float lg_bandwidth = 0.0;

static bool lg_is_converting = false;
static char lg_convert_base_path[256];
static uint32_t lg_convert_frame_num = 0;

static void kokuyoseki_callback(struct timeval time, int button, int value);

static void release(void *user_data) {
	free(user_data);
}

static int get_last_id(const char *path) {
	int last_id = 0;
	struct dirent *d;
	DIR *dir;

	dir = opendir(path);
	while ((d = readdir(dir)) != 0) {
		if (d->d_name[0] != L'.') {
			int id = 0;
			sscanf(d->d_name, "%d", &id);
			if (id > last_id) {
				last_id = id;
			}
		}
	}
	return last_id;
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
							" video_delay=\"%f\""
							" />", light0_value, light1_value, motor0_value,
					motor1_value, motor2_value, motor3_value, lg_video_delay);
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

#define NUM_OF_CAM 2
static float lg_fps[NUM_OF_CAM] = { };
static int lg_frameskip[NUM_OF_CAM] = { };

static void parse_xml(char *xml) {
	char *q_str = NULL;
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

static void status_handler(unsigned char *data, int data_len) {
	if (data[0] == 0xFF && data[1] == 0xE1) { //xmp
		int xmp_len = 0;
		xmp_len = ((unsigned char*) data)[2] << 8;
		xmp_len += ((unsigned char*) data)[3];

		char *xml = (char*) data + strlen((char*) data) + 1;
		parse_xml(xml);
	}
}

static float sub_angle(float a, float b) {
	float v = a - b;
	v -= floor(v / 360) * 360;
	if (v > 180.0) {
		v -= 360.0;
	}
	return v;
}

static void packet_menu_convert_node_callback(struct _MENU_T *menu,
		enum MENU_EVENT event);
static void loading_callback(void *user_data, int ret) {
	if (lg_is_converting) {
		MENU_T *menu = (MENU_T*) user_data;
		menu->selected = false;
		packet_menu_convert_node_callback(menu, MENU_EVENT_DESELECTED);
	}
	printf("end of loading\n");
}

void *transmit_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "DA TRANSMIT");

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
				// end of pid control
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

		xmp_len = picam360_driver_xmp(buff, sizeof(buff), lg_light_value[0],
				lg_light_value[1], lg_motor_value[0], lg_motor_value[1],
				lg_motor_value[2], lg_motor_value[3]);

		rtp_sendpacket((unsigned char*) buff, xmp_len, PT_CMD);

		last_time = time;
		usleep(100 * 1000); //less than 10Hz
	} // end of while
}

static void command_handler(void *user_data, const char *_buff) {
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
	} else if (strncmp(cmd, PLUGIN_NAME ".set_video_delay", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value = 0;
			sscanf(param, "%f", &value);
			lg_video_delay = MAX(MIN(value,MAX_DELAY_COUNT), 0);
			printf("set_video_delay : completed\n");
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
			rtp_start_loading(param, true, true,
					(RTP_LOADING_CALLBACK) loading_callback, NULL);
			printf("start_loading : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".stop_loading", sizeof(buff)) == 0) {
		rtp_stop_loading();
		printf("stop_loading : completed\n");
	} else if (strncmp(cmd, PLUGIN_NAME ".n", sizeof(buff)) == 0) {
		struct timeval time;
		gettimeofday(&time, NULL);
		kokuyoseki_callback(time, NEXT_BUTTON, 0);
	} else if (strncmp(cmd, PLUGIN_NAME ".b", sizeof(buff)) == 0) {
		struct timeval time;
		gettimeofday(&time, NULL);
		kokuyoseki_callback(time, BACK_BUTTON, 0);
	} else if (strncmp(cmd, PLUGIN_NAME ".m", sizeof(buff)) == 0) {
		struct timeval time;
		gettimeofday(&time, NULL);
		kokuyoseki_callback(time, BLACKOUT_BUTTON, 0);
	} else {
		printf(":unknown command : %s\n", buff);
	}
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
	switch (node_id) {
	case PICAM360_HOST_NODE_ID:
		switch (event_id) {
		case PICAM360_CAPTURE_EVENT_AFTER_SNAP:
			if (lg_is_converting) {
				rtp_increment_loading(100 * 1000); //10 fps
				lg_convert_frame_num++;

				char dst[256];
				snprintf(dst, 256, "%s/%d.jpeg", lg_convert_base_path,
						lg_convert_frame_num);
				lg_plugin_host->snap(4096, 2048, RENDERING_MODE_EQUIRECTANGULAR,
						dst);
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void kokuyoseki_callback(struct timeval time, int button, int value) {
	if (value == 1) {
		return;
	}
	struct timeval diff;
	timersub(&time, &lg_last_kokuyoseki_time, &diff);
	float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
	MENU_T *menu = lg_plugin_host->get_menu();
	if (!menu->activated) {
		switch (button) {
		case NEXT_BUTTON:
			lg_thrust += 1;
			printf("thrust %f\n", lg_thrust);
			break;
		case BACK_BUTTON:
			lg_thrust -= 1;
			printf("thrust %f\n", lg_thrust);
			break;
		case BLACKOUT_BUTTON:
			if (!lg_plugin_host->get_menu_visible()) {
				lg_plugin_host->set_menu_visible(true);
			} else {
				menu_operate(menu, MENU_OPERATE_ACTIVE_NEXT); //open menu
			}
			break;
		case BACK_BUTTON_LONG:
			lg_plugin_host->set_menu_visible(false);
			break;
		}
		{
			float *quat;
			switch (button) {
			case NEXT_BUTTON:
			case BACK_BUTTON:
				quat = lg_plugin_host->get_view_quatanion();
				if (quat) {
					memcpy(lg_target_quatanion, quat,
							sizeof(lg_target_quatanion));
				}
			}
		}
	} else if (!lg_plugin_host->get_menu_visible()) {
		switch (button) {
		case BLACKOUT_BUTTON:
			lg_plugin_host->set_menu_visible(true);
			break;
		case NEXT_BUTTON:
			switch (lg_ui_mode) {
			case UI_MODE_LIGHT:
				lg_light_strength += 1;
				break;
			case UI_MODE_FOV:
				lg_plugin_host->set_fov(lg_plugin_host->get_fov() - 1);
				break;
			default:
				break;
			}
			break;
		case BACK_BUTTON:
			switch (lg_ui_mode) {
			case UI_MODE_LIGHT:
				lg_light_strength -= 1;
				break;
			case UI_MODE_FOV:
				lg_plugin_host->set_fov(lg_plugin_host->get_fov() + 1);
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
	} else {
		switch (button) {
		case BACK_BUTTON_LONG:
			lg_plugin_host->set_menu_visible(false);
			break;
		case NEXT_BUTTON:
			menu_operate(menu, MENU_OPERATE_SELECT);
			break;
		case BACK_BUTTON:
			menu_operate(menu, MENU_OPERATE_DESELECT);
			break;
		case BLACKOUT_BUTTON:
			menu_operate(menu, MENU_OPERATE_ACTIVE_NEXT);
			break;
		default:
			break;
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
	lg_video_delay = json_number_value(
			json_object_get(options, PLUGIN_NAME ".video_delay"));
}

static void save_options(void *user_data, json_t *options) {
	json_object_set_new(options, PLUGIN_NAME ".p_gain", json_real(lg_p_gain));
	json_object_set_new(options, PLUGIN_NAME ".i_gain", json_real(lg_i_gain));
	json_object_set_new(options, PLUGIN_NAME ".d_gain", json_real(lg_d_gain));
	json_object_set_new(options, PLUGIN_NAME ".video_delay",
			json_real(lg_video_delay));
}

static int rtp_callback(unsigned char *data, unsigned int data_len,
		unsigned char pt, unsigned int seq_num) {
	if (data_len == 0) {
		return -1;
	}
	static unsigned int last_seq_num = 0;
	if (seq_num != last_seq_num + 1) {
		printf("packet lost : from %d to %d\n", last_seq_num, seq_num);
	}
	last_seq_num = seq_num;

	if (pt == PT_STATUS) {
		status_handler((unsigned char*) data, data_len);
	} else if (pt == PT_CAM_BASE + 0) {
		if (lg_plugin_host && lg_plugin_host->decode_video) {
			lg_plugin_host->decode_video(0, (unsigned char*) data, data_len);
		}
	} else if (pt == PT_CAM_BASE + 1) {
		if (lg_plugin_host && lg_plugin_host->decode_video) {
			lg_plugin_host->decode_video(1, (unsigned char*) data, data_len);
		}
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
	if (lg_pid_enabled) {
		cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur, L"\nyaw=%f,\tpitch=%f\tpid_value=%f\tdelta_value=%f\n",
				yaw, pitch, lg_pid_value[0],
				lg_delta_pid_target[0][0]);
		for (int i = 0; i < MOTOR_NUM; i++) {
			cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur, L"m%d=%d, ", i, lg_motor_value[i]);
		}
	}
	return lg_info;
}

static void packet_menu_save_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED:
		break;
	case MENU_EVENT_DEACTIVATED:
		break;
	case MENU_EVENT_SELECTED:
		if (!rtp_is_recording(NULL) && !rtp_is_loading(NULL)) {
			char dst[256];
			int last_id = get_last_id(PACKET_FOLDER_PATH);
			snprintf(dst, 256, PACKET_FOLDER_PATH "/%d.rtp", last_id + 1);
			rtp_start_recording(dst);
			printf("start recording %s\n", dst);
		}
		break;
	case MENU_EVENT_DESELECTED:
		if (rtp_is_recording(NULL)) {
			rtp_stop_recording();
			printf("stop recording\n");
		}
		break;
	case MENU_EVENT_BEFORE_DELETE:
		break;
	case MENU_EVENT_NONE:
	default:
		break;
	}
}

static void packet_menu_load_node_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED:
		break;
	case MENU_EVENT_DEACTIVATED:
		break;
	case MENU_EVENT_SELECTED:
		if (!rtp_is_recording(NULL) && !rtp_is_loading(NULL)) {
			char name[256];
			snprintf(name, 256, PACKET_FOLDER_PATH "/%s",
					(char*) menu->user_data);
			rtp_start_loading(name, true, true,
					(RTP_LOADING_CALLBACK) loading_callback, NULL);
			printf("start loading %s\n", name);
			lg_plugin_host->set_menu_visible(false);
		}
		break;
	case MENU_EVENT_DESELECTED:
		if (rtp_is_loading(NULL)) {
			rtp_stop_loading();
			printf("stop loading\n");
		}
		break;
	case MENU_EVENT_BEFORE_DELETE:
		free(menu->user_data);
		break;
	case MENU_EVENT_NONE:
	default:
		break;
	}
}

static void packet_menu_load_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED:
		if (1) {
			struct dirent *d;
			DIR *dir;

			dir = opendir(PACKET_FOLDER_PATH);
			while ((d = readdir(dir)) != 0) {
				if (d->d_name[0] != L'.') {
					char *name_s = malloc(256);
					wchar_t name[256];
					snprintf(name_s, 256, "%s", d->d_name);
					swprintf(name, 256, L"%s", d->d_name);
					MENU_T *node_menu = menu_new(name,
							packet_menu_load_node_callback, name_s);
					menu_add_submenu(menu, node_menu, INT_MAX);
				}
			}
		}
		break;
	case MENU_EVENT_DEACTIVATED:
		for (int idx = 0; menu->submenu[idx]; idx++) {
			menu_delete(&menu->submenu[idx]);
		}
		break;
	case MENU_EVENT_SELECTED:
		break;
	case MENU_EVENT_DESELECTED:
		break;
	case MENU_EVENT_BEFORE_DELETE:
		break;
	case MENU_EVENT_NONE:
	default:
		break;
	}
}

static void packet_menu_convert_node_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED:
		break;
	case MENU_EVENT_DEACTIVATED:
		break;
	case MENU_EVENT_SELECTED:
		if (!rtp_is_recording(NULL) && !rtp_is_loading(NULL)) {
			char src[256];
			snprintf(src, 256, PACKET_FOLDER_PATH "/%s",
					(char*) menu->user_data);
			bool succeeded = rtp_start_loading(src, false, false,
					(RTP_LOADING_CALLBACK) loading_callback, menu);
			if (succeeded) {
				snprintf(lg_convert_base_path, 256, VIDEO_FOLDER_PATH "/%s",
						(char*) menu->user_data);
				int ret = mkdir(lg_convert_base_path,
						S_IRUSR | S_IWUSR | S_IXUSR | /* rwx */
						S_IRGRP | S_IWGRP | S_IXGRP | /* rwx */
						S_IROTH | S_IXOTH | S_IXOTH);
				if (ret == 0 || errno == EEXIST) {
					char dst[256];
					snprintf(dst, 256, "%s/%d.jpeg", lg_convert_base_path,
							lg_convert_frame_num);
					lg_plugin_host->snap(4096, 2048,
							RENDERING_MODE_EQUIRECTANGULAR, dst);
					lg_is_converting = true;

					printf("start converting %s to %s\n", src,
							lg_convert_base_path);
				} else {
					succeeded = false;
				}
			}
			if (!succeeded) {
				menu->selected = false;
				packet_menu_convert_node_callback(menu, MENU_EVENT_DESELECTED);
			}
		}
		break;
	case MENU_EVENT_DESELECTED:
		if (rtp_is_loading(NULL)) {
			rtp_stop_loading();
			lg_is_converting = false;

			printf("stop converting\n");
		}
		break;
	case MENU_EVENT_BEFORE_DELETE:
		free(menu->user_data);
		break;
	case MENU_EVENT_NONE:
	default:
		break;
	}
}

static void packet_menu_convert_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED:
		if (1) {
			struct dirent *d;
			DIR *dir;

			dir = opendir(PACKET_FOLDER_PATH);
			while ((d = readdir(dir)) != 0) {
				if (d->d_name[0] != L'.') {
					char *name_s = malloc(256);
					wchar_t name[256];
					snprintf(name_s, 256, "%s", d->d_name);
					swprintf(name, 256, L"%s", d->d_name);
					MENU_T *node_menu = menu_new(name,
							packet_menu_convert_node_callback, name_s);
					menu_add_submenu(menu, node_menu, INT_MAX);
				}
			}
		}
		break;
	case MENU_EVENT_DEACTIVATED:
		for (int idx = 0; menu->submenu[idx]; idx++) {
			menu_delete(&menu->submenu[idx]);
		}
		break;
	case MENU_EVENT_SELECTED:
		break;
	case MENU_EVENT_DESELECTED:
		break;
	case MENU_EVENT_BEFORE_DELETE:
		break;
	case MENU_EVENT_NONE:
	default:
		break;
	}
}
static void function_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		switch ((int) menu->user_data) {
		case 0:
			menu->selected = false;
			{
				char dst[256];
				int last_id = get_last_id(STILL_FOLDER_PATH);
				snprintf(dst, 256, STILL_FOLDER_PATH "/%d.jpeg", last_id + 1);
				lg_plugin_host->snap(4096, 2048, RENDERING_MODE_EQUIRECTANGULAR,
						dst);
			}
			break;
		default:
			break;
		}
		break;
	case MENU_EVENT_DESELECTED:
		lg_ui_mode = UI_MODE_DEFAULT;
		break;
	default:
		break;
	}
}
static void mode_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		lg_ui_mode = (enum UI_MODE) menu->user_data;
		lg_plugin_host->set_menu_visible(false);
		break;
	case MENU_EVENT_DESELECTED:
		lg_ui_mode = UI_MODE_DEFAULT;
		break;
	default:
		break;
	}
}
static void pid_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		lg_pid_enabled = (bool) menu->user_data;
		menu->parent->submenu[0]->marked = lg_pid_enabled;
		menu->parent->submenu[1]->marked = !lg_pid_enabled;
		menu->selected = false;
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}
static void system_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		if ((enum SYSTEM_CMD) menu->user_data == SYSTEM_CMD_SHUTDOWN) {
			exit(1);
		}
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

void create_driver_agent(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	init();
	lg_plugin_host = plugin_host;

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(PLUGIN_T));
		strcpy(plugin->name, PLUGIN_NAME);
		plugin->release = release;
		plugin->command_handler = command_handler;
		plugin->event_handler = event_handler;
		plugin->init_options = init_options;
		plugin->save_options = save_options;
		plugin->get_info = get_info;
		plugin->user_data = plugin;

		*_plugin = plugin;
	}
	{			//menu
		MENU_T *menu = lg_plugin_host->get_menu();
		{
			MENU_T *function_menu = menu_new(L"Function", NULL, NULL);
			MENU_T *function_snap_menu = menu_new(L"Snap",
					function_menu_callback, (void*) 0);
			menu_add_submenu(function_menu, function_snap_menu, INT_MAX);
			menu_add_submenu(menu, function_menu, INT_MAX);		//add main menu
		}
		{
			MENU_T *mode_menu = menu_new(L"Mode", NULL, NULL);
			MENU_T *mode_light_menu = menu_new(L"Light", mode_menu_callback,
					(void*) UI_MODE_LIGHT);
			MENU_T *mode_fov_menu = menu_new(L"Fov", mode_menu_callback,
					(void*) UI_MODE_FOV);
			menu_add_submenu(mode_menu, mode_light_menu, INT_MAX);
			menu_add_submenu(mode_menu, mode_fov_menu, INT_MAX);
			menu_add_submenu(menu, mode_menu, INT_MAX);		//add main menu
		}
		{
			MENU_T *pid_menu = menu_new(L"PID", NULL, NULL);
			MENU_T *pid_on_menu = menu_new(L"On", pid_menu_callback,
					(void*) true);
			MENU_T *pid_off_menu = menu_new(L"Off", pid_menu_callback,
					(void*) false);
			pid_off_menu->marked = true;
			menu_add_submenu(pid_menu, pid_on_menu, INT_MAX);
			menu_add_submenu(pid_menu, pid_off_menu, INT_MAX);
			menu_add_submenu(menu, pid_menu, INT_MAX);		//add main menu
		}
		{
			MENU_T *packet_menu = menu_new(L"Packet", NULL, NULL);
			MENU_T *packet_save_menu = menu_new(L"Save",
					packet_menu_save_callback, NULL);
			MENU_T *packet_load_menu = menu_new(L"Load",
					packet_menu_load_callback, NULL);
			MENU_T *packet_convert_menu = menu_new(L"Convert",
					packet_menu_convert_callback, NULL);
			menu_add_submenu(packet_menu, packet_save_menu, INT_MAX);
			menu_add_submenu(packet_menu, packet_load_menu, INT_MAX);
			menu_add_submenu(packet_menu, packet_convert_menu, INT_MAX);
			menu_add_submenu(menu, packet_menu, INT_MAX);		//add main menu
		}
		{
			MENU_T *system_menu = menu_new(L"System", NULL, NULL);
			MENU_T *system_shutdown_menu = menu_new(L"Shutdown",
					system_menu_callback, (void*) SYSTEM_CMD_SHUTDOWN);
			menu_add_submenu(system_menu, system_shutdown_menu, INT_MAX);
			menu_add_submenu(menu, system_menu, INT_MAX);		//add main menu
		}
	}
}
