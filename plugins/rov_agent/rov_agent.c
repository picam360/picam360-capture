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

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PLUGIN_NAME "rov_agent"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

#define LIGHT_NUM 2
#define MOTOR_NUM 4
static float lg_light_value[LIGHT_NUM] = { 0, 0 };
static float lg_motor_value[MOTOR_NUM] = { 0, 0, 0, 0 };
static float lg_light_strength = 0; //0 to 100
static float lg_thrust = 0; //-100 to 100
static float lg_brake_ps = 5; // percent
static VECTOR4D_T lg_target_quaternion = { .ary = { 0, 0, 0, 1 } };

static bool lg_is_compass_calib = false;
static float lg_compass_min[3] = { INT_MAX, INT_MAX, INT_MAX };
static float lg_compass_max[3] = { -INT_MAX, -INT_MAX, -INT_MAX };

static bool lg_lowlevel_control = false;
static bool lg_pid_enabled = false;
static float lg_yaw_diff = 0;
static float lg_pitch_diff = 0;
static float lg_p_gain = 1.0;
static float lg_i_gain = 1.0;
static float lg_d_gain = 1.0;
static float lg_pid_value[3] = { }; //x, z, delta yaw
static float lg_delta_pid_target[3] = { }; //x, z, delta yaw

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
		char cmd[256];
		sprintf(cmd, "upstream.mpu9250.start_compass_calib");
		lg_plugin_host->send_command(cmd);

		printf("start_compass_calib : completed\n");
	} else if (strncmp(cmd, PLUGIN_NAME ".stop_compass_calib", sizeof(buff))
			== 0) {
		char cmd[256];
		sprintf(cmd, "upstream.mpu9250.stop_compass_calib");
		lg_plugin_host->send_command(cmd);

		printf("stop_compass_calib : completed\n");
	} else if (strncmp(cmd, PLUGIN_NAME ".set_pid_enabled", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);
			{
				char cmd[256];
				sprintf(cmd, "upstream.rov_driver.set_pid_enabled %d",
						value ? 1 : 0);
				lg_plugin_host->send_command(cmd);
			}
			printf("set_pid_enabled : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_light_strength", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);
			{
				char cmd[256];
				sprintf(cmd, "upstream.rov_driver.set_light_strength %f",
						value);
				lg_plugin_host->send_command(cmd);
			}
			printf("set_light_strength : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".increment_thrust", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float v;
			int num = sscanf(param, "%f", &v);

			if (num == 1) {
				param = strtok(NULL, " \n");
				if (param != NULL) {
					float x, y, z, w;
					int num = sscanf(param, "%f,%f,%f,%f", &x, &y, &z, &w);

					if (num == 4) {
						char cmd[256];
						sprintf(cmd,
								"upstream.rov_driver.increment_thrust %f %f,%f,%f,%f",
								v, x, y, z, w);
						lg_plugin_host->send_command(cmd);
					}
				} else {
					char cmd[256];
					sprintf(cmd, "upstream.rov_driver.increment_thrust %f", v);
					lg_plugin_host->send_command(cmd);
				}
			}
			printf("increment_thrust : completed\n");
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
	float increment = 0;
	switch (node_id) {
	case PICAM360_CONTROLLER_NODE_ID:
		switch (event_id) {
		case PICAM360_CONTROLLER_EVENT_NEXT:
			increment = 1;
			break;
		case PICAM360_CONTROLLER_EVENT_BACK:
			increment = -1;
			break;
		default:
			break;
		}
		{
			char cmd[256];
			VECTOR4D_T quat = lg_plugin_host->get_view_quaternion();
			sprintf(cmd, "rov_agent.increment_thrust %f %f,%f,%f,%f", increment,
					quat.x, quat.y, quat.z, quat.w);
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
	cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur,
			L"thr %.1f, pch=%.1f, yaw=%.1f, mtr=%.1f", lg_thrust, lg_pitch_diff,
			lg_yaw_diff, lg_motor_value[0]);
	for (int i = 1; i < MOTOR_NUM; i++) {
		cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur, L",%.1f",
				lg_motor_value[i]);
	}
	if (lg_pid_enabled) {
		cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur,
				L"\npid_value=%.1f,%.1f,%.1f\tdelta_value=%.1f,%.1f,%.1f",
				lg_pid_value[0], lg_pid_value[1], lg_pid_value[2],
				lg_delta_pid_target[0], lg_delta_pid_target[1],
				lg_delta_pid_target[2]);
	}
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
		if (1) {
			bool value = !(bool) menu->user_data;
			menu->user_data = (void*) value;
			if (value) {
				swprintf(menu->name, 8, L"On");
			} else {
				swprintf(menu->name, 8, L"Off");
			}
			{
				char cmd[256];
				sprintf(cmd, "rov_agent.set_pid_enabled %d", value ? 1 : 0);
				lg_plugin_host->send_command(cmd);
			}
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
			{
				char cmd[256];
				sprintf(cmd, "rov_agent.set_light_strength %f",
						lg_light_strength);
				lg_plugin_host->send_command(cmd);
			}
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

#define STATUS_VAR(name) lg_status_ ## name
#define WATCH_INIT(plugin_host, prefix, name) STATUS_VAR(name) = new_status(prefix #name); \
                                               (plugin_host)->add_watch(STATUS_VAR(name));

static STATUS_T *STATUS_VAR(is_compass_calib);
static STATUS_T *STATUS_VAR(compass_min);
static STATUS_T *STATUS_VAR(compass_max);
static STATUS_T *STATUS_VAR(light_value);
static STATUS_T *STATUS_VAR(motor_value);
static STATUS_T *STATUS_VAR(light_strength);
static STATUS_T *STATUS_VAR(brake_ps);
static STATUS_T *STATUS_VAR(thrust);
static STATUS_T *STATUS_VAR(target_quaternion);
static STATUS_T *STATUS_VAR(lowlevel_control);
static STATUS_T *STATUS_VAR(pid_enabled);
static STATUS_T *STATUS_VAR(yaw_diff);
static STATUS_T *STATUS_VAR(pitch_diff);
static STATUS_T *STATUS_VAR(p_gain);
static STATUS_T *STATUS_VAR(i_gain);
static STATUS_T *STATUS_VAR(d_gain);
static STATUS_T *STATUS_VAR(pid_value);
static STATUS_T *STATUS_VAR(delta_pid_target);

static void status_release(void *user_data) {
	free(user_data);
}
static void status_get_value(void *user_data, char *buff, int buff_len) {
	//STATUS_T *status = (STATUS_T*) user_data;
}
static void status_set_value(void *user_data, const char *value) {
	STATUS_T *status = (STATUS_T*) user_data;
	if (status == STATUS_VAR(is_compass_calib)) {
		int v = 0;
		sscanf(value, "%d", &v);
		lg_is_compass_calib = (v != 0);
	} else if (status == STATUS_VAR(compass_min)) {
		sscanf(value, "%f,%f,%f", &lg_compass_min[0], &lg_compass_min[1],
				&lg_compass_min[2]);
	} else if (status == STATUS_VAR(compass_max)) {
		sscanf(value, "%f,%f,%f", &lg_compass_max[0], &lg_compass_max[1],
				&lg_compass_max[2]);
	} else if (status == STATUS_VAR(light_value)) {
		sscanf(value, "%f,%f", &lg_light_value[0], &lg_light_value[1]);
	} else if (status == STATUS_VAR(motor_value)) {
		sscanf(value, "%f,%f,%f,%f", &lg_motor_value[0], &lg_motor_value[1],
				&lg_motor_value[2], &lg_motor_value[3]);
	} else if (status == STATUS_VAR(light_strength)) {
		sscanf(value, "%f", &lg_light_strength);
	} else if (status == STATUS_VAR(brake_ps)) {
		sscanf(value, "%f", &lg_brake_ps);
	} else if (status == STATUS_VAR(thrust)) {
		sscanf(value, "%f", &lg_thrust);
	} else if (status == STATUS_VAR(target_quaternion)) {
		sscanf(value, "%f,%f,%f,%f", &lg_target_quaternion.x,
				&lg_target_quaternion.y, &lg_target_quaternion.z,
				&lg_target_quaternion.w);
	} else if (status == STATUS_VAR(lowlevel_control)) {
		int v = 0;
		sscanf(value, "%d", &v);
		lg_lowlevel_control = (v != 0);
	} else if (status == STATUS_VAR(pid_enabled)) {
		int v = 0;
		sscanf(value, "%d", &v);
		lg_pid_enabled = (v != 0);
	} else if (status == STATUS_VAR(yaw_diff)) {
		sscanf(value, "%f", &lg_yaw_diff);
	} else if (status == STATUS_VAR(pitch_diff)) {
		sscanf(value, "%f", &lg_pitch_diff);
	} else if (status == STATUS_VAR(p_gain)) {
		sscanf(value, "%f", &lg_p_gain);
	} else if (status == STATUS_VAR(i_gain)) {
		sscanf(value, "%f", &lg_i_gain);
	} else if (status == STATUS_VAR(d_gain)) {
		sscanf(value, "%f", &lg_d_gain);
	} else if (status == STATUS_VAR(pid_value)) {
		sscanf(value, "%f,%f,%f", &lg_pid_value[0], &lg_pid_value[1],
				&lg_pid_value[2]);
	} else if (status == STATUS_VAR(delta_pid_target)) {
		sscanf(value, "%f,%f,%f", &lg_delta_pid_target[0],
				&lg_delta_pid_target[1], &lg_pid_value[2]);
	}
}

static STATUS_T *new_status(const char *name) {
	STATUS_T *status = (STATUS_T*) malloc(sizeof(STATUS_T));
	strcpy(status->name, name);
	status->get_value = status_get_value;
	status->set_value = status_set_value;
	status->release = status_release;
	status->user_data = status;
	return status;
}

static void init_status() {
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "mpu9250.", is_compass_calib);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "mpu9250.", compass_min);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "mpu9250.", compass_max);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", light_value);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", motor_value);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", light_strength);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", brake_ps);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", thrust);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", target_quaternion);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", lowlevel_control);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", pid_enabled);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", yaw_diff);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", pitch_diff);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", p_gain);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", i_gain);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", d_gain);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", pid_value);
	WATCH_INIT(lg_plugin_host, UPSTREAM_DOMAIN "rov_driver.", delta_pid_target);
}

#endif //status block

static bool is_init = false;
static void init(PLUGIN_HOST_T *plugin_host) {
	if (is_init) {
		return;
	}
	is_init = true;

	lg_plugin_host = plugin_host;

	_init_menu();
	init_status();
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	init(plugin_host);

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(PLUGIN_T));
		memset(plugin, 0, sizeof(PLUGIN_T));
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
