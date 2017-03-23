#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

#include "MotionSensor.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static float lg_compass_min[3] = { -317.000000, -416.000000, -208.000000 };
//static float lg_compass_min[3] = { INT_MAX, INT_MAX, INT_MAX };
static float lg_compass_max[3] = { 221.000000, -67.000000, 98.000000 };
//static float lg_compass_max[3] = { -INT_MAX, -INT_MAX, -INT_MAX };
static float lg_compass[4] = { };
static float lg_quat[4] = { };
static float lg_north = 0;
static int lg_north_count = 0;

void *threadFunc(void *data) {

	do {
		ms_update();

		{ //compas : calibration
			float calib[3];
			float bias[3];
			float gain[3];
			for (int i = 0; i < 3; i++) {
				lg_compass_min[i] = MIN(lg_compass_min[i], compass[i]);
				lg_compass_max[i] = MAX(lg_compass_max[i], compass[i]);
				bias[i] = (lg_compass_min[i] + lg_compass_max[i]) / 2;
				gain[i] = (lg_compass_max[i] - lg_compass_min[i]) / 2;
				calib[i] = (compass[i] - bias[i])
						/ (gain[i] == 0 ? 1 : gain[i]);
			}
			float norm = sqrt(
					calib[0] * calib[0] + calib[1] * calib[1]
							+ calib[2] * calib[2]);
			for (int i = 0; i < 3; i++) {
				calib[i] /= norm;
			}
			lg_compass[0] = calib[1];
			lg_compass[1] = -calib[0];
			lg_compass[2] = -calib[2];
			lg_compass[3] = 1.0;
		}
		{ //quat : convert from mpu coodinate to opengl coodinate
			lg_quat[0] = quatanion[1];	//x
			lg_quat[1] = quatanion[3];	//y : swap y and z
			lg_quat[2] = -quatanion[2];	//z : swap y and z
			lg_quat[3] = quatanion[0];	//w
		}
		{ //north
			float north = 0;

			float view_matrix[16];
			mat4_fromQuat(view_matrix, lg_quat);
			mat4_invert(view_matrix, view_matrix);

			float compass_mat[16] = { };
			memcpy(compass_mat, lg_compass, sizeof(float) * 4);

			mat4_transpose(compass_mat, compass_mat);
			mat4_multiply(compass_mat, compass_mat, view_matrix);
			mat4_transpose(compass_mat, compass_mat);

			north = -atan2(compass_mat[2], compass_mat[0]) * 180 / M_PI;

			lg_north = (lg_north * lg_north_count + north)
					/ (lg_north_count + 1);
			lg_north_count++;
			if (lg_north_count > 1000) {
				lg_north_count = 1000;
			}
		}

		usleep(5000);
	} while (1);
}

static bool is_init = false;

void init_mpu9250() {
	if (is_init) {
		return;
	} else {
		is_init = true;
	}

	ms_open();

	//do{
	//	ms_update();
	//	printf("%f,%f,%f,%f\n",  (float)_q[0] / (1<<30),  (float)_q[1] / (1<<30),  (float)_q[2] / (1<<30),  (float)_q[3] / (1<<30));
	//	usleep(5000);
	//}while(1);
	pthread_t f1_thread;
	pthread_create(&f1_thread, NULL, threadFunc, NULL);
}

float *get_quatanion_mpu9250() {
	return lg_quat;
}

float *get_compass_mpu9250() {
	return lg_compass;
}

float get_temperature_c_mpu9250() {
	return (temp - 32) * 5 / 9;
}

float get_north_mpu9250() {
	return lg_north;
}
