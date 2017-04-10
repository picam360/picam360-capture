#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include <string.h>

QUATERNION get_quaternion_from_x(float rad) {
	QUATERNION q;
	q.x = sin(rad * 0.5);
	q.y = 0;
	q.z = 0;
	q.w = cos(rad * 0.5);
	return q;
}

QUATERNION get_quaternion_from_y(float rad) {
	QUATERNION q;
	q.x = 0;
	q.y = sin(rad * 0.5);
	q.z = 0;
	q.w = cos(rad * 0.5);
	return q;
}

QUATERNION get_quaternion_from_z(float rad) {
	QUATERNION q;
	q.x = 0;
	q.y = 0;
	q.z = sin(rad * 0.5);
	q.w = cos(rad * 0.5);
	return q;
}

QUATERNION multiply_quaternion(QUATERNION q1, QUATERNION q2) {
	QUATERNION q;
	q.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
	q.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
	q.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
	q.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
	return q;
}
