#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <float.h>
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

#include "usv_driver.h"

#include <mat4/identity.h>
#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/invert.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PLUGIN_NAME "usv_driver"

#define PT_STATUS 100
#define PT_CMD 101
#define PT_CAM_BASE 110

static PLUGIN_HOST_T *lg_plugin_host = NULL;

#define LIGHT_NUM 2
#define MOTOR_NUM 4
static float lg_motor_center = 1500;
static float lg_motor_margin = 25;
static float lg_motor_range = 100;
#define MOTOR_BASE(value) lg_motor_center + lg_motor_margin * ((value < 0.5 && value > -0.5) ? 0 : (value > 0) ? 1 : -1)

static int lg_light_id[LIGHT_NUM] = { 4, 34 };
static int lg_motor_id[MOTOR_NUM] = { 17, 18, 27, 24 };

static float lg_light_value[LIGHT_NUM] = { 0, 0 };
static float lg_motor_value[MOTOR_NUM] = { 0, 0, 0, 0 };
static float lg_motor_dir[4] = { 1, 1, 1, 1 };

static float lg_light_strength = 0; //0 to 100
static float lg_thrust = 0; //-100 to 100
static float lg_rudder = 0; //-100 to 100
static float lg_target_heading = 0; //-180 to 180
static float lg_max_rpm = 6;
static int lg_thruster_mode = 0; //0:single, 1:double, 2:quad

#define PID_NUM 2
static bool lg_lowlevel_control = false;
static bool lg_pid_enabled = false;
static bool lg_heading_lock = true;
static float lg_pid_gain[PID_NUM][3] = { { 1.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 } };
static float lg_pid_value[PID_NUM] = { }; //[rpm, heading]
static float lg_delta_pid_target[3][PID_NUM * 2 + 1] = { }; //[history][rpm, rpm_lpf, heading, heading_lpf t]

static void release(void *user_data) {
	free(user_data);
}

static void update_pwm() {
	int len = 0;
	char cmd[256] = { };
	int fd = open("/dev/pi-blaster", O_WRONLY);
	if (fd < 0) {
		return;
	}

	{
		float value = lg_light_value[0];
		value = pow(value / 100, 3);
		len = sprintf(cmd, "%d=%f\n", lg_light_id[0], value);
		write(fd, cmd, len);
	}

	{
		float value = lg_light_value[1];
		value = pow(value / 100, 3);
		len = sprintf(cmd, "%d=%f\n", lg_light_id[1], value);
		write(fd, cmd, len);
	}

	{
		float value = lg_motor_value[0];
		value = lg_motor_dir[0] * (value / 100) * lg_motor_range + MOTOR_BASE(lg_motor_dir[0] * value);
		len = sprintf(cmd, "%d=%fus\n", lg_motor_id[0], value);
		write(fd, cmd, len);
	}

	{
		float value = lg_motor_value[1];
		value = lg_motor_dir[1] * (value / 100) * lg_motor_range + MOTOR_BASE(lg_motor_dir[1] * value);
		len = sprintf(cmd, "%d=%fus\n", lg_motor_id[1], value);
		write(fd, cmd, len);
	}

	{
		float value = lg_motor_value[2];
		value = lg_motor_dir[2] * (value / 100) * lg_motor_range + MOTOR_BASE(lg_motor_dir[2] * value);
		len = sprintf(cmd, "%d=%fus\n", lg_motor_id[2], value);
		write(fd, cmd, len);
	}

	{
		float value = lg_motor_value[3];
		value = lg_motor_dir[3] * (value / 100) * lg_motor_range + MOTOR_BASE(lg_motor_dir[3] * value);
		len = sprintf(cmd, "%d=%fus\n", lg_motor_id[3], value);
		write(fd, cmd, len);
	}

	close(fd);
}

static float normalize_angle(float v) {
	v -= floor(v / 360) * 360;
	if (v < -180.0) {
		v += 360.0;
	}
	if (v > 180.0) {
		v -= 360.0;
	}
	return v;
}

static bool lg_debugdump = false;
void *pid_control_single(float t_s, float north) {
	lg_motor_value[0] = lg_thrust;
	lg_motor_value[1] = lg_rudder;
	return NULL;
}
void *pid_control_double(float t_s, float north) {
	if (!lg_pid_enabled) {
		lg_motor_value[0] = lg_thrust + lg_rudder / 2;
		lg_motor_value[1] = lg_thrust - lg_rudder / 2;
		return NULL;
	}

	static float last_t_s = -1;
	if (last_t_s < 0) {
		last_t_s = t_s;
		return NULL;
	}
	float diff_sec = t_s - last_t_s;
	if (diff_sec < 0.01) {
		return NULL;
	}
	last_t_s = t_s;

	static float lpf_gain1 = 0.8;
	static float lpf_gain2 = 0.2;
	static float heading_lpf1 = FLT_MIN;
	static float last_heading_lpf1 = FLT_MIN;
	static float heading_lpf2 = FLT_MIN;
	static float last_heading_lpf2 = FLT_MIN;
	float heading = -north; //clockwise
	if (heading_lpf1 == FLT_MIN) {
		heading_lpf1 = heading;
		last_heading_lpf1 = heading;
		heading_lpf2 = heading;
		last_heading_lpf2 = heading;
		return NULL;
	}
	heading_lpf1 = normalize_angle(heading_lpf1 + normalize_angle(heading - heading_lpf1) * lpf_gain1);
	heading_lpf2 = normalize_angle(heading_lpf2 + normalize_angle(heading - heading_lpf2) * lpf_gain2);

	float rpm_lpf1 = normalize_angle(heading_lpf1 - last_heading_lpf1) / 360 / diff_sec * 60;
	float rpm_lpf2 = normalize_angle(heading_lpf2 - last_heading_lpf2) / 360 / diff_sec * 60;
	float target_rpm = lg_rudder * lg_max_rpm / 100;
	lg_delta_pid_target[0][0] = rpm_lpf1 - target_rpm;
	lg_delta_pid_target[0][1] = rpm_lpf2 - target_rpm;
	lg_delta_pid_target[0][2] = normalize_angle(heading_lpf1 - lg_target_heading);
	lg_delta_pid_target[0][3] = normalize_angle(heading_lpf2 - lg_target_heading);
	lg_delta_pid_target[0][PID_NUM * 2] = t_s;
	last_heading_lpf1 = heading_lpf1;
	last_heading_lpf2 = heading_lpf2;

	if (lg_delta_pid_target[2][PID_NUM * 2] == 0) { //skip
		//increment
		for (int j = 3 - 1; j >= 1; j--) {
			for (int k = 0; k < PID_NUM * 2 + 1; k++) {
				lg_delta_pid_target[j][k] = lg_delta_pid_target[j - 1][k];
			}
		}
		return NULL;
	}

	bool is_angle[PID_NUM] = { false, true };
	for (int k = 0; k < PID_NUM; k++) {
		float p_value = lg_pid_gain[k][0] * lg_delta_pid_target[0][k * 2];
		float diff = (lg_delta_pid_target[0][k * 2 + 1] - lg_delta_pid_target[1][k * 2 + 1]);
		if (is_angle[k]) {
			diff = normalize_angle(diff);
		}
		float d_value = lg_pid_gain[k][2] * diff / diff_sec;
		float delta_value = p_value + d_value;
		lg_pid_value[k] = delta_value;
	}
	//increment
	for (int j = 3 - 1; j >= 1; j--) {
		for (int k = 0; k < PID_NUM * 2 + 1; k++) {
			lg_delta_pid_target[j][k] = lg_delta_pid_target[j - 1][k];
		}
	}
	{		//limit
		lg_pid_value[0] = MIN(MAX(lg_pid_value[0], -200), 200);		//rpm to thruster differencial
		lg_pid_value[1] = MIN(MAX(lg_pid_value[1], -200), 200);		//heading to thruster differencial
	}

	if (lg_debugdump) {
		printf("vehicle t=%.3fs: rpm=%.3f, %.3f, %.3f : heading=%.3f, %.3f, %.3f\n", diff_sec, rpm_lpf1, target_rpm, lg_pid_value[0], heading_lpf1, lg_target_heading, lg_pid_value[1]);
	}

	{		//aply
		float lpf_gain = 0.5;
		float value = (lg_heading_lock ? lg_pid_value[1] : lg_pid_value[0]);
		lg_motor_value[0] = lg_motor_value[0] * (1 - lpf_gain) + (lg_thrust + value / 2) * lpf_gain;
		lg_motor_value[1] = lg_motor_value[1] * (1 - lpf_gain) + (lg_thrust - value / 2) * lpf_gain;
		return NULL;
	}
}
void *pid_control_quad(float t_s, float north) {

}
void *pid_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "DA PID");

	while (1) {
		usleep(50 * 1000); //less than 20Hz
		if (lg_lowlevel_control) {
			continue;
		}

		float north = 0;
		VECTOR4D_T quat = { };
		MPU_T *mpu = lg_plugin_host->get_mpu();
		if (mpu) {
			north = mpu->get_north(mpu);
			quat = mpu->get_quaternion(mpu);
		}
		//cal
		//trancate min max
		lg_light_strength = MIN(MAX(lg_light_strength, 0), 100);
		lg_light_value[0] = lg_light_strength;
		lg_light_value[1] = lg_light_strength;

		//trancate min max
		lg_thrust = MIN(MAX(lg_thrust, -100), 100);
		lg_rudder = MIN(MAX(lg_rudder, -100), 100);
		lg_target_heading = normalize_angle(lg_target_heading);

		switch (lg_thruster_mode) {
		case 0:
			pid_control_single(quat.t, north);
			break;
		case 1:
			pid_control_double(quat.t, north);
			break;
		case 2:
			pid_control_quad(quat.t, north);
			break;
		}
		update_pwm();
	} // end of while
}

static bool is_init_pwm = false;
static void init_pwm() {
	if (is_init_pwm) {
		return;
	} else {
		is_init_pwm = true;
	}
	int fd = open("/dev/pi-blaster", O_WRONLY);
	if (fd > 0) {
		char cmd[256];
		int len;
		len = sprintf(cmd, "%d=%f\n", lg_light_id[0], 0.0f);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_light_id[1], 0.0f);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%fus\n", lg_motor_id[0], lg_motor_center);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%fus\n", lg_motor_id[1], lg_motor_center);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%fus\n", lg_motor_id[2], lg_motor_center);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%fus\n", lg_motor_id[3], lg_motor_center);
		write(fd, cmd, len);
		close(fd);
	}

	pthread_t pid_thread;
	pthread_create(&pid_thread, NULL, pid_thread_func, (void*) NULL);
}

static int command_handler(void *user_data, const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, PLUGIN_NAME ".set_thrust", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float v1, v2, v3,v4;
			int num = sscanf(param, "%f,%f,%f,%f", &v1, &v2, &v3, &v4);

			if (num >= 1) {
				lg_thrust = v1;
			}
			if (num >= 2 && !lg_heading_lock) {
				lg_rudder = v2;
			}
			if (num >= 3) {
				lg_target_heading = v3;
			}
			if (num >= 4) {
				lg_pid_enabled = (v4 != 0);
			}
			if (lg_debugdump) {
				printf("%s : completed\n", buff);
			}
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_thruster_mode", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float v;
			int num = sscanf(param, "%f", &v);

			if (num >= 1) {
				lg_thruster_mode = (int) v;
			}
			printf("set_thruster_mode : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_light_strength", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float v;
			int num = sscanf(param, "%f", &v);

			if (num == 1) {
				lg_light_strength = v;
			}
			printf("set_light_strength : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_lowlevel_control", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_lowlevel_control = (value != 0);
			printf("set_lowlevel_control : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_pid_enabled", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_pid_enabled = (value != 0);
			lg_thrust = 0;
			lg_rudder = 0;
			memset(lg_pid_value, 0, sizeof(lg_pid_value));
			memset(lg_delta_pid_target, 0, sizeof(lg_delta_pid_target));

			printf("set_pid_enabled : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_heading_lock", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_heading_lock = (value != 0);

			printf("set_heading_lock : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_light_value", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int id = 0;
			float value = 0;
			sscanf(param, "%d=%f", &id, &value);
			if (id < LIGHT_NUM) {
				lg_light_value[id] = value;
			}
			sscanf(param, "%f", &value);
			printf("set_light_value : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_motor_value", sizeof(buff)) == 0) {
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
	} else {
		printf(":unknown command : %s\n", buff);
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
	switch (node_id) {
	case PICAM360_HOST_NODE_ID:
		switch (event_id) {
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void init_options(void *user_data, json_t *options) {
	{ //pid_gain
		json_t *ary1 = json_object_get(options, PLUGIN_NAME ".pid_gain");
		if (json_is_array(ary1)) {
			int size1 = json_array_size(ary1);
			for (int i1 = 0; i1 < MIN(size1, PID_NUM); i1++) {
				json_t *ary2 = json_array_get(ary1, i1);
				if (json_is_array(ary2)) {
					int size2 = json_array_size(ary2);
					for (int i2 = 0; i2 < MIN(size2, 3); i2++) {
						lg_pid_gain[i1][i2] = json_number_value(json_array_get(ary2, i2));
					}
				}
			}
		}
	}

	for (int i = 0; i < LIGHT_NUM; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".light%d_id", i);
		int id = (int) json_number_value(json_object_get(options, buff));
		if (id != 0) {
			lg_light_id[i] = id;
		}
	}

	for (int i = 0; i < MOTOR_NUM; i++) {
		char buff[256];
		int value;
		sprintf(buff, PLUGIN_NAME ".motor%d_id", i);
		value = (int) json_number_value(json_object_get(options, buff));
		if (value != 0) {
			lg_motor_id[i] = value;
		}
		sprintf(buff, PLUGIN_NAME ".motor%d_dir", i);
		value = (int) json_number_value(json_object_get(options, buff));
		if (value != 0) {
			lg_motor_dir[i] = value;
		}
	}
	{
		float value;
		value = json_number_value(json_object_get(options, PLUGIN_NAME ".motor_center"));
		if (value != 0) {
			lg_motor_center = value;
		}
		value = json_number_value(json_object_get(options, PLUGIN_NAME ".motor_margin"));
		if (value != 0) {
			lg_motor_margin = value;
		}
		value = json_number_value(json_object_get(options, PLUGIN_NAME ".motor_range"));
		if (value != 0) {
			lg_motor_range = value;
		}
	}
	lg_thruster_mode = (int) json_number_value(json_object_get(options, PLUGIN_NAME ".thruster_mode"));

	init_pwm(); //need motor ids
}

static void save_options(void *user_data, json_t *options) {
	{ //pid_gain
		json_t *ary1 = json_array();
		for (int i1 = 0; i1 < PID_NUM; i1++) {
			json_t *ary2 = json_array();
			for (int i2 = 0; i2 < 3; i2++) {
				json_array_append_new(ary2, json_real(lg_pid_gain[i1][i2]));
			}
			json_array_append_new(ary1, ary2);
		}
		json_object_set_new(options, PLUGIN_NAME ".pid_gain", ary1);
	}

	for (int i = 0; i < LIGHT_NUM; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".light%d_id", i);
		json_object_set_new(options, buff, json_real(lg_light_id[i]));
	}

	for (int i = 0; i < MOTOR_NUM; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".motor%d_id", i);
		json_object_set_new(options, buff, json_real(lg_motor_id[i]));
		sprintf(buff, PLUGIN_NAME ".motor%d_dir", i);
		json_object_set_new(options, buff, json_real(lg_motor_dir[i]));
	}

	{
		json_object_set_new(options, PLUGIN_NAME ".motor_center", json_real(lg_motor_center));
		json_object_set_new(options, PLUGIN_NAME ".motor_margin", json_real(lg_motor_margin));
		json_object_set_new(options, PLUGIN_NAME ".motor_range", json_real(lg_motor_range));
	}
	json_object_set_new(options, PLUGIN_NAME ".thruster_mode", json_real(lg_thruster_mode));
}

#define MAX_INFO_LEN 1024
static char lg_info[MAX_INFO_LEN];
static char *get_info(void *user_data) {
	lg_info[0] = L'\0';
	return lg_info;
}

#if (1) //status block

#define STATUS_VAR(name) lg_status_ ## name
#define STATUS_INIT(plugin_host, prefix, name) STATUS_VAR(name) = new_status(prefix #name); \
                                               (plugin_host)->add_status(STATUS_VAR(name));

static STATUS_T *STATUS_VAR(light_value);
static STATUS_T *STATUS_VAR(motor_value);
static STATUS_T *STATUS_VAR(light_strength);
static STATUS_T *STATUS_VAR(thrust);
static STATUS_T *STATUS_VAR(lowlevel_control);
static STATUS_T *STATUS_VAR(pid_enabled);
static STATUS_T *STATUS_VAR(pid_value);
static STATUS_T *STATUS_VAR(delta_pid_target);

static void status_release(void *user_data) {
	free(user_data);
}
static void status_get_value(void *user_data, char *buff, int buff_len) {
	STATUS_T *status = (STATUS_T*) user_data;
	if (status == STATUS_VAR(light_value)) {
		snprintf(buff, buff_len, "%f,%f", lg_light_value[0], lg_light_value[1]);
	} else if (status == STATUS_VAR(motor_value)) {
		snprintf(buff, buff_len, "%f,%f,%f,%f", lg_motor_value[0], lg_motor_value[1], lg_motor_value[2], lg_motor_value[3]);
	} else if (status == STATUS_VAR(light_strength)) {
		snprintf(buff, buff_len, "%f", lg_light_strength);
	} else if (status == STATUS_VAR(thrust)) {
		snprintf(buff, buff_len, "%f", lg_thrust);
	} else if (status == STATUS_VAR(lowlevel_control)) {
		snprintf(buff, buff_len, "%d", lg_lowlevel_control ? 1 : 0);
	} else if (status == STATUS_VAR(pid_enabled)) {
		snprintf(buff, buff_len, "%d", lg_pid_enabled ? 1 : 0);
	} else if (status == STATUS_VAR(pid_value)) {
		snprintf(buff, buff_len, "%f,%f,%f", lg_pid_value[0], lg_pid_value[1], lg_pid_value[2]);
	} else if (status == STATUS_VAR(delta_pid_target)) {
		snprintf(buff, buff_len, "%f,%f,%f", lg_delta_pid_target[0][0], lg_delta_pid_target[0][1], lg_delta_pid_target[0][2]);
	}
}

static void status_set_value(void *user_data, const char *value) {
//STATUS_T *status = (STATUS_T*) user_data;
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
//	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", light_value);
//	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", motor_value);
//	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", light_strength);
//	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", thrust);
//	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", lowlevel_control);
//	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", pid_enabled);
//	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", pid_value);
//	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", delta_pid_target);
}

#endif //status block

static bool is_init = false;
static void init(PLUGIN_HOST_T *plugin_host) {
	if (is_init) {
		return;
	} else {
		is_init = true;
	}
	lg_plugin_host = plugin_host;

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
