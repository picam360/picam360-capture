#include <unistd.h>
#include <pthread.h>

#include "MotionSensor.h"

static float quat[4];

void *threadFunc(void *data) {

	do {
		ms_update();

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
	quat[0] = -quatanion[2];
	quat[1] = quatanion[3];
	quat[2] = -quatanion[1];
	quat[3] = quatanion[0];
	return quat;
}
