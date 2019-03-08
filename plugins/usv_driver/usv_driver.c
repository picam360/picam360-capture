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

#include "rov_driver.h"

#include <mat4/identity.h>
#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/invert.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PLUGIN_NAME "rov_driver"

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
static int lg_motor_id[MOTOR_NUM] = { 18, 36, 35, 17 };

static float lg_light_value[LIGHT_NUM] = { 0, 0 };
static float lg_motor_value[MOTOR_NUM] = { 0, 0, 0, 0 };
static float lg_motor_dir[4] = { -1, 1, -1, 1 };

static float lg_light_strength = 0; //0 to 100
static float lg_thrust = 0; //-100 to 100
static float lg_brake_ps = 5; // percent
static VECTOR4D_T lg_target_quaternion = { .ary = { 0, 0, 0, 1 } };
static VECTOR4D_T lg_offset_quaternion = { .ary = { 0, 0, 0, 1 } };

static bool lg_lowlevel_control = false;
static bool lg_pid_enabled = false;
static float lg_yaw_diff = 0;
static float lg_pitch_diff = 0;
static float lg_p_gain = 1.0;
static float lg_i_gain = 1.0;
static float lg_d_gain = 1.0;
static float lg_pid_value[3] = { }; //x, z, delta yaw
static float lg_delta_pid_target[3][4] = { }; //[history][x, z, delta yaw, t]

static float lg_offset_yaw = 0.0;
static float lg_offset_pitch = 0.0;
static float lg_offset_roll = 0.0;

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

static float sub_angle(float a, float b) {
	float v = a - b;
	v -= floor(v / 360) * 360;
	if (v > 180.0) {
		v -= 360.0;
	}
	return v;
}
static VECTOR4D_T _get_quaternion() {
	VECTOR4D_T quat = { };
	MPU_T *mpu = lg_plugin_host->get_mpu();
	if (mpu) {
		quat = mpu->get_quaternion(mpu);
		quat = quaternion_multiply(quat, lg_offset_quaternion); // Rc=RcRcoRc-1Rc=RcRco
	}
	return quat;
}

static bool lg_debugdump = false;
void *pid_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "DA PID");

	static float last_time = -1;
	while (1) {
		usleep(50 * 1000); //less than 20Hz
		if (lg_lowlevel_control) {
			continue;
		}

		VECTOR4D_T quat = _get_quaternion();
		if (last_time < 0) { //init last_time
			last_time = quat.t;
			continue;
		}
		//quat = quaternion_multiply(quat, quaternion_get_from_z(M_PI)); //mpu offset
		//(RcRt-1Rc-1)*(Rc)*vtg, target coordinate will be converted into camera coordinate
		float vtg[16] = { 0, -1, 0, 1 }; // looking at ground
		float unif_matrix[16];
		float camera_matrix[16];
		float target_matrix[16];
		mat4_identity(unif_matrix);
		mat4_identity(camera_matrix);
		mat4_identity(target_matrix);
		mat4_fromQuat(camera_matrix, quat.ary);
		mat4_fromQuat(target_matrix, lg_target_quaternion.ary);
		mat4_invert(target_matrix, target_matrix);
		mat4_multiply(unif_matrix, unif_matrix, target_matrix); // Rt-1
		mat4_multiply(unif_matrix, unif_matrix, camera_matrix); // RcRt-1

		mat4_transpose(vtg, vtg);
		mat4_multiply(vtg, vtg, unif_matrix);
		mat4_transpose(vtg, vtg);

		float xz = sqrt(vtg[0] * vtg[0] + vtg[2] * vtg[2]);
		lg_yaw_diff = -atan2(vtg[2], vtg[0]) * 180 / M_PI;
		lg_pitch_diff = atan2(xz, -vtg[1]) * 180 / M_PI; //[-180:180]

		//time
		float diff_sec = quat.t - last_time;
		if (diff_sec < 1E-6) { //<1us need to wait
			continue;
		}
		//cal
		//trancate min max
		lg_light_strength = MIN(MAX(lg_light_strength, 0), 100);
		lg_light_value[0] = lg_light_strength;
		lg_light_value[1] = lg_light_strength;

		//trancate min max
		lg_thrust = MIN(MAX(lg_thrust, -100), 100);
		//brake
		lg_thrust *= exp(log(1.0 - lg_brake_ps / 100) * diff_sec);

		if (lg_pid_enabled) {
			float x, y, z;
			if (lg_debugdump) {
				quaternion_get_euler(quat, &y, &x, &z, EULER_SEQUENCE_YXZ);
				printf("vehicle : %f, %f, %f\n", x * 180 / M_PI, y * 180 / M_PI, z * 180 / M_PI);
			}
			if (lg_debugdump) {
				quaternion_get_euler(lg_target_quaternion, &y, &x, &z, EULER_SEQUENCE_YXZ);
				printf("target  : %f, %f, %f\n", x * 180 / M_PI, y * 180 / M_PI, z * 180 / M_PI);
			}

			if (lg_debugdump) {
				printf("vehicle : %f, %f, %f\n", vtg[0], vtg[1], vtg[2]);
			}

			static float last_yaw = 0;
			lg_delta_pid_target[0][0] = cos(lg_yaw_diff * M_PI / 180) * (lg_pitch_diff / 180); // x [-1:1]
			lg_delta_pid_target[0][1] = sin(lg_yaw_diff * M_PI / 180) * (lg_pitch_diff / 180); // z [-1:1]
			lg_delta_pid_target[0][2] = sub_angle(lg_yaw_diff, last_yaw) / 180; // delta yaw [-1:1]
			lg_delta_pid_target[0][3] = quat.t;

			diff_sec = lg_delta_pid_target[0][3] - lg_delta_pid_target[1][3];
			diff_sec = MAX(MIN(diff_sec, 1.0), 0.001);

			for (int k = 0; k < 3; k++) {
				float p_value = lg_p_gain * (lg_delta_pid_target[0][k] - lg_delta_pid_target[1][k]);
				float i_value = lg_i_gain * lg_delta_pid_target[0][k] * diff_sec;
				float d_value = lg_d_gain * (lg_delta_pid_target[0][k] - 2 * lg_delta_pid_target[1][k] + lg_delta_pid_target[2][k]) / diff_sec;
				float delta_value = p_value + i_value + d_value;
				lg_pid_value[k] += delta_value;
				lg_pid_value[k] = MIN(MAX(lg_pid_value[k], -2500), 2500);
			}

			//increment
			for (int j = 3 - 1; j >= 1; j--) {
				for (int k = 0; k < 4; k++) {
					lg_delta_pid_target[j][k] = lg_delta_pid_target[j - 1][k];
				}
			}
			last_yaw = lg_yaw_diff;

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
					float power_calib = (lg_pid_value[k] > 0 ? 1 : -1) * sqrt(abs(lg_pid_value[k]));
					motor_pid_value[i] += power_calib * motor_pid_gain[k][i];
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
		update_pwm();

		last_time = quat.t;
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
	} else if (strncmp(cmd, PLUGIN_NAME ".increment_thrust", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float v;
			int num = sscanf(param, "%f", &v);

			if (num == 1) {
				lg_thrust += v;
				param = strtok(NULL, " \n");
				if (param != NULL) {
					float x, y, z, w;
					int num = sscanf(param, "%f,%f,%f,%f", &x, &y, &z, &w);

					if (num == 4) {
						lg_target_quaternion.x = x;
						lg_target_quaternion.y = y;
						lg_target_quaternion.z = z;
						lg_target_quaternion.w = w;
					}
				}
			}
			printf("set_thrust : completed\n");
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
	} else if (strncmp(cmd, PLUGIN_NAME ".set_target_quaternion", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float x, y, z, w;
			int num = sscanf(param, "%f,%f,%f,%f", &x, &y, &z, &w);

			if (num == 4) {
				lg_target_quaternion.x = x;
				lg_target_quaternion.y = y;
				lg_target_quaternion.z = z;
				lg_target_quaternion.w = w;
			}
			printf("set_target_quaternion : completed\n");
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
			lg_target_quaternion = _get_quaternion();
			memset(lg_pid_value, 0, sizeof(lg_pid_value));
			memset(lg_delta_pid_target, 0, sizeof(lg_delta_pid_target));

			printf("set_pid_enabled : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_p_gain", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_p_gain = value;
			lg_thrust = 0;
			memset(lg_pid_value, 0, sizeof(lg_pid_value));
			memset(lg_delta_pid_target, 0, sizeof(lg_delta_pid_target));

			printf("set_p_gain : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_i_gain", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_i_gain = value;
			lg_thrust = 0;
			memset(lg_pid_value, 0, sizeof(lg_pid_value));
			memset(lg_delta_pid_target, 0, sizeof(lg_delta_pid_target));

			printf("set_i_gain : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_d_gain", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_d_gain = value;
			lg_thrust = 0;
			memset(lg_pid_value, 0, sizeof(lg_pid_value));
			memset(lg_delta_pid_target, 0, sizeof(lg_delta_pid_target));

			printf("set_d_gain : completed\n");
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
	lg_p_gain = json_number_value(json_object_get(options, PLUGIN_NAME ".p_gain"));
	lg_i_gain = json_number_value(json_object_get(options, PLUGIN_NAME ".i_gain"));
	lg_d_gain = json_number_value(json_object_get(options, PLUGIN_NAME ".d_gain"));

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
	{
		lg_offset_yaw = json_number_value(json_object_get(options, PLUGIN_NAME ".offset_yaw"));
		lg_offset_pitch = json_number_value(json_object_get(options, PLUGIN_NAME ".offset_pitch"));
		lg_offset_roll = json_number_value(json_object_get(options, PLUGIN_NAME ".offset_roll"));

		lg_offset_quaternion = quaternion_init();
		lg_offset_quaternion = quaternion_multiply(lg_offset_quaternion, quaternion_get_from_z(lg_offset_roll));
		lg_offset_quaternion = quaternion_multiply(lg_offset_quaternion, quaternion_get_from_x(lg_offset_pitch));
		lg_offset_quaternion = quaternion_multiply(lg_offset_quaternion, quaternion_get_from_y(lg_offset_yaw));
	}

	init_pwm(); //need motor ids
}

static void save_options(void *user_data, json_t *options) {
	json_object_set_new(options, PLUGIN_NAME ".p_gain", json_real(lg_p_gain));
	json_object_set_new(options, PLUGIN_NAME ".i_gain", json_real(lg_i_gain));
	json_object_set_new(options, PLUGIN_NAME ".d_gain", json_real(lg_d_gain));

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
	{
		json_object_set_new(options, PLUGIN_NAME ".offset_yaw", json_real(lg_offset_yaw));
		json_object_set_new(options, PLUGIN_NAME ".offset_pitch", json_real(lg_offset_pitch));
		json_object_set_new(options, PLUGIN_NAME ".offset_roll", json_real(lg_offset_roll));
	}
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
	STATUS_T *status = (STATUS_T*) user_data;
	if (status == STATUS_VAR(light_value)) {
		snprintf(buff, buff_len, "%f,%f", lg_light_value[0], lg_light_value[1]);
	} else if (status == STATUS_VAR(motor_value)) {
		snprintf(buff, buff_len, "%f,%f,%f,%f", lg_motor_value[0], lg_motor_value[1], lg_motor_value[2], lg_motor_value[3]);
	} else if (status == STATUS_VAR(light_strength)) {
		snprintf(buff, buff_len, "%f", lg_light_strength);
	} else if (status == STATUS_VAR(brake_ps)) {
		snprintf(buff, buff_len, "%f", lg_brake_ps);
	} else if (status == STATUS_VAR(thrust)) {
		snprintf(buff, buff_len, "%f", lg_thrust);
	} else if (status == STATUS_VAR(target_quaternion)) {
		snprintf(buff, buff_len, "%f,%f,%f,%f", lg_target_quaternion.x, lg_target_quaternion.y, lg_target_quaternion.z, lg_target_quaternion.w);
	} else if (status == STATUS_VAR(lowlevel_control)) {
		snprintf(buff, buff_len, "%d", lg_lowlevel_control ? 1 : 0);
	} else if (status == STATUS_VAR(pid_enabled)) {
		snprintf(buff, buff_len, "%d", lg_pid_enabled ? 1 : 0);
	} else if (status == STATUS_VAR(yaw_diff)) {
		snprintf(buff, buff_len, "%f", lg_yaw_diff);
	} else if (status == STATUS_VAR(pitch_diff)) {
		snprintf(buff, buff_len, "%f", lg_pitch_diff);
	} else if (status == STATUS_VAR(p_gain)) {
		snprintf(buff, buff_len, "%f", lg_p_gain);
	} else if (status == STATUS_VAR(i_gain)) {
		snprintf(buff, buff_len, "%f", lg_i_gain);
	} else if (status == STATUS_VAR(d_gain)) {
		snprintf(buff, buff_len, "%f", lg_d_gain);
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
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", light_value);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", motor_value);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", light_strength);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", brake_ps);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", thrust);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", target_quaternion);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", lowlevel_control);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", pid_enabled);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", yaw_diff);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", pitch_diff);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", p_gain);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", i_gain);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", d_gain);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", pid_value);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", delta_pid_target);
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
