#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include "quaternion.h"

QUATERNION_T quaternion_init() {
	QUATERNION_T q;
	q.x = 0;
	q.y = 0;
	q.z = 0;
	q.w = 1;
	return q;
}

QUATERNION_T quaternion_get_from_x(float rad) {
	QUATERNION_T q;
	q.x = sin(rad * 0.5);
	q.y = 0;
	q.z = 0;
	q.w = cos(rad * 0.5);
	return q;
}

QUATERNION_T quaternion_get_from_y(float rad) {
	QUATERNION_T q;
	q.x = 0;
	q.y = sin(rad * 0.5);
	q.z = 0;
	q.w = cos(rad * 0.5);
	return q;
}

QUATERNION_T quaternion_get_from_z(float rad) {
	QUATERNION_T q;
	q.x = 0;
	q.y = 0;
	q.z = sin(rad * 0.5);
	q.w = cos(rad * 0.5);
	return q;
}

QUATERNION_T quaternion_multiply(QUATERNION_T q1, QUATERNION_T q2) {
	QUATERNION_T q;
	q.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
	q.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
	q.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
	q.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
	return q;
}
