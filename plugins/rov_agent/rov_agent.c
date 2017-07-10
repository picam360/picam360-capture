#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <wchar.h>
#include <limits.h>
#include <dirent.h>
#include <pthread.h>

#include "rov_agent.h"

#include <mat4/identity.h>
#include <mat4/rotateY.h>
#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/invert.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PLUGIN_NAME "driver_agent"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

#define LIGHT_NUM 2
#define MOTOR_NUM 4
static int lg_light_value[LIGHT_NUM] = { 0, 0 };
static int lg_motor_value[MOTOR_NUM] = { 0, 0, 0, 0 };
static float lg_light_strength = 0; //0 to 100
static float lg_thrust = 0; //-100 to 100
//static float lg_brake_ps = 5; // percent

static bool lg_is_compass_calib = false;
static float lg_compass_min[3] = { INT_MAX, INT_MAX, INT_MAX };
static float lg_compass_max[3] = { -INT_MAX, -INT_MAX, -INT_MAX };

static bool lg_pid_enabled = false;

static void release(void *user_data) {
	free(user_data);
}

static int command_handler(void *user_data, const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, PLUGIN_NAME ".start_compass_calib", sizeof(buff))
			== 0) {
		lg_is_compass_calib = true;
		char cmd[256];
		sprintf(cmd, "upstream.picam360_driver.start_compass_calib");
		lg_plugin_host->send_command(cmd);

		printf("start_compass_calib : completed\n");
	} else if (strncmp(cmd, PLUGIN_NAME ".stop_compass_calib", sizeof(buff))
			== 0) {
		lg_is_compass_calib = false;
		char cmd[256];
		sprintf(cmd, "upstream.picam360_driver.stop_compass_calib");
		lg_plugin_host->send_command(cmd);

		printf("stop_compass_calib : completed\n");
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

			char cmd[256];
			sprintf(cmd, "upstream.picam360_driver.set_video_delay %f", value);
			lg_plugin_host->send_command(cmd);

			printf("set_video_delay : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_offset_x", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int cam_num = 0;
			float value = 0;
			sscanf(param, "%d=%f", &cam_num, &value);

			char cmd[256];
			sprintf(cmd, "upstream.picam360_driver.add_camera_offset_x %d=%f",
					cam_num, value);
			lg_plugin_host->send_command(cmd);

			printf("add_camera_offset_x : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_offset_y", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int cam_num = 0;
			float value = 0;
			sscanf(param, "%d=%f", &cam_num, &value);

			char cmd[256];
			sprintf(cmd, "upstream.picam360_driver.add_camera_offset_y %d=%f",
					cam_num, value);
			lg_plugin_host->send_command(cmd);

			printf("add_camera_offset_y : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_offset_yaw", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int cam_num = 0;
			float value = 0;
			sscanf(param, "%d=%f", &cam_num, &value);

			char cmd[256];
			sprintf(cmd, "upstream.picam360_driver.add_camera_offset_yaw %d=%f",
					cam_num, value);
			lg_plugin_host->send_command(cmd);

			printf("add_camera_offset_yaw : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_horizon_r", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int cam_num = 0;
			float value = 0;
			sscanf(param, "%d=%f", &cam_num, &value);

			char cmd[256];
			sprintf(cmd, "upstream.picam360_driver.add_camera_horizon_r %d=%f",
					cam_num, value);
			lg_plugin_host->send_command(cmd);

			printf("add_camera_horizon_r : completed\n");
		}
	} else {
		printf(":unknown command : %s\n", buff);
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
	switch (node_id) {
	case PICAM360_CONTROLLER_NODE_ID:
		switch (event_id) {
		case PICAM360_CONTROLLER_EVENT_NEXT:
			lg_thrust++;
			break;
		case PICAM360_CONTROLLER_EVENT_BACK:
			lg_thrust--;
			break;
		default:
			break;
		}
		{
			char cmd[256];
			VECTOR4D_T quat = lg_plugin_host->get_view_quaternion();
			sprintf(cmd, "upstream.picam360_driver.set_thrust %f %f,%f,%f,%f",
					lg_thrust, quat.x, quat.y, quat.z, quat.w);
			lg_plugin_host->send_command(cmd);
		}
		break;
	default:
		break;
	}
}

static void init_options(void *user_data, json_t *options) {
}

static void save_options(void *user_data, json_t *options) {
}

#define MAX_INFO_LEN 1024
static wchar_t lg_info[MAX_INFO_LEN];
static wchar_t *get_info(void *user_data) {
	int cur = 0;
	float north;
	VECTOR4D_T quat = lg_plugin_host->get_camera_quaternion(-1);
	quaternion_get_euler(quat, &north, NULL, NULL, EULER_SEQUENCE_YXZ);
//	cur += swprintf(lg_info, MAX_INFO_LEN,
//			L"N %.1f, rx %.1f Mbps, fps %.1f:%.1f skip %d:%d",
//			north * 180 / M_PI, lg_bandwidth, lg_fps[0], lg_fps[1],
//			lg_frameskip[0], lg_frameskip[1]);
//	if (rtp_is_recording(NULL)) {
//		char *path;
//		rtp_is_recording(&path);
//		cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur, L", to %hs", path);
//	}
//	if (rtp_is_loading(NULL)) {
//		char *path;
//		rtp_is_loading(&path);
//		cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur, L", from %hs", path);
//	}
//	if (lg_pid_enabled) {
//		cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur,
//				L"\npitch=%fyaw=%f,\t\tpid_value=%f\tdelta_value=%f\n",
//				lg_pitch_diff, lg_yaw_diff, lg_pid_value[0],
//				lg_delta_pid_target[0][0]);
//		for (int i = 0; i < MOTOR_NUM; i++) {
//			cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur, L"m%d=%d, ", i,
//					lg_motor_value[i]);
//		}
//	}
	if (lg_is_compass_calib) {
		cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur,
				L"\ncompass calib : min[%.1f,%.1f,%.1f] max[%.1f,%.1f,%.1f]",
				lg_compass_min[0], lg_compass_min[1], lg_compass_min[2],
				lg_compass_max[0], lg_compass_max[1], lg_compass_max[2]);
	}
	return lg_info;
}

///////////////////////////////////////////////////////
#if (1) //menu block

static void pid_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		lg_thrust = 0;
		memset(lg_motor_value, 0, sizeof(lg_motor_value));
//		lg_yaw_diff = 0;
//		lg_pitch_diff = 0;
//		memset(lg_pid_value, 0, sizeof(lg_pid_value));
		lg_pid_enabled = !(bool) menu->user_data;
		menu->user_data = (void*) lg_pid_enabled;
		if (lg_pid_enabled) {
			swprintf(menu->name, 8, L"On");
		} else {
			swprintf(menu->name, 8, L"Off");
		}
		menu->selected = false;
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

static void light_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		if (1) {
			int value = (int) menu->user_data;
			if (value == -1 || value == 1) {
				value = lg_light_strength + value;
			}
			lg_light_strength = MIN(MAX(value, 0), 100);
		}
		menu->selected = false;
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

static void _init_menu() {
	MENU_T *sub_menu = menu_get_submenu(lg_plugin_host->get_menu(), L"Config",
			true);
	MENU_T *pid_menu = menu_add_submenu(sub_menu, menu_new(L"PID", NULL, NULL),
			INT_MAX);
	{
		MENU_T *sub_menu = pid_menu;
		MENU_T *off_menu = menu_add_submenu(sub_menu,
				menu_new(L"Off", pid_menu_callback, (void*) false), INT_MAX);
		off_menu->marked = true;
	}
	MENU_T *light_menu = menu_add_submenu(sub_menu,
			menu_new(L"Light", NULL, NULL), INT_MAX);
	{
		MENU_T *sub_menu = light_menu;
		menu_add_submenu(sub_menu,
				menu_new(L"MIN", light_menu_callback, (void*) 0), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new(L"-", light_menu_callback, (void*) -1), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new(L"+", light_menu_callback, (void*) 1), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new(L"MAX", light_menu_callback, (void*) 100), INT_MAX);
	}
}

#endif //menu block

#if (1) //status block

static void status_release(void *user_data) {
	free(user_data);
}
static void status_get_value(void *user_data, char *buff, int buff_len) {
	//STATUS_T *status = (STATUS_T*) user_data;
}
static void status_set_value(void *user_data, const char *value) {
	STATUS_T *status = (STATUS_T*) user_data;
	if (strcmp(status->name, "compass_min") == 0) {
		sscanf(value, "%f,%f,%f", &lg_compass_min[0], &lg_compass_min[1],
				&lg_compass_min[2]);
	} else if (strcmp(status->name, "compass_max") == 0) {
		sscanf(value, "%f,%f,%f", &lg_compass_max[0], &lg_compass_max[1],
				&lg_compass_max[2]);
	}
}

static void init_status() {
	{
		STATUS_T *status = (STATUS_T*) malloc(sizeof(STATUS_T));
		strcpy(status->name, "compass_min");
		status->get_value = status_get_value;
		status->set_value = status_set_value;
		status->release = status_release;

		lg_plugin_host->add_watch(status);
	}
	{
		STATUS_T *status = (STATUS_T*) malloc(sizeof(STATUS_T));
		strcpy(status->name, "compass_max");
		status->get_value = status_get_value;
		status->set_value = status_set_value;
		status->release = status_release;

		lg_plugin_host->add_watch(status);
	}
}

#endif //status block

static bool is_init = false;
static void init() {
	if (is_init) {
		return;
	}
	is_init = true;

	_init_menu();
	init_status();
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	lg_plugin_host = plugin_host;
	init();

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
}
