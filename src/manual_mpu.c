#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <limits.h>

#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/invert.h>

#include "manual_mpu.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define MPU_NAME "manual"

typedef struct _MPU_T_INTERNAL {
	MPU_T super;
	VECTOR4D_T compass;
	VECTOR4D_T quat;
	float north;
	float temp;
} MPU_T_INTERNAL;

static VECTOR4D_T get_quaternion(void *user_data) {
	MPU_T_INTERNAL *mpu = (MPU_T_INTERNAL*) user_data;
	return mpu->quat;
}

static void set_quaternion(void *user_data, VECTOR4D_T value) {
	MPU_T_INTERNAL *mpu = (MPU_T_INTERNAL*) user_data;
	mpu->quat = value;
}

static VECTOR4D_T get_compass(void *user_data) {
	MPU_T_INTERNAL *mpu = (MPU_T_INTERNAL*) user_data;
	return mpu->compass;
}

static float get_temperature(void *user_data) {
	MPU_T_INTERNAL *mpu = (MPU_T_INTERNAL*) user_data;
	return mpu->temp;
}

static float get_north(void *user_data) {
	MPU_T_INTERNAL *mpu = (MPU_T_INTERNAL*) user_data;
	return mpu->north;
}

static void release(void *user_data) {
	free(user_data);
}

static void create_mpu(void *user_data, MPU_T **_mpu) {
	MPU_T_INTERNAL *mpu = (MPU_T_INTERNAL*) malloc(sizeof(MPU_T_INTERNAL));
	memset(mpu, 0, sizeof(MPU_T_INTERNAL));
	strcpy(mpu->super.name, MPU_NAME);
	mpu->super.release = release;
	mpu->super.get_quaternion = get_quaternion;
	mpu->super.set_quaternion = set_quaternion;
	mpu->super.get_compass = get_compass;
	mpu->super.get_temperature = get_temperature;
	mpu->super.get_north = get_north;
	mpu->super.user_data = mpu;

	mpu->compass.w = 1.0;
	mpu->quat.w = 1.0;

	*_mpu = (MPU_T*)mpu;
}

void create_manual_mpu_factory(MPU_FACTORY_T **_mpu_factory) {
	MPU_FACTORY_T *mpu_factory = (MPU_FACTORY_T*) malloc(sizeof(MPU_FACTORY_T));
	memset(mpu_factory, 0, sizeof(MPU_FACTORY_T));
	strcpy(mpu_factory->name, MPU_NAME);
	mpu_factory->release = release;
	mpu_factory->create_mpu = create_mpu;
	mpu_factory->user_data = mpu_factory;

	*_mpu_factory = mpu_factory;
}
