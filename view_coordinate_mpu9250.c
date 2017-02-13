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

void init_device_mpu9250() {

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
	quat[0] = -(float) _q[2] / (1 << 30);
	quat[1] = (float) _q[3] / (1 << 30);
	quat[2] = -(float) _q[1] / (1 << 30);
	quat[3] = (float) _q[0] / (1 << 30);
	return quat;
}
