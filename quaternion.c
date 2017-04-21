#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include "quaternion.h"

VECTOR4D_T quaternion_init() {
	VECTOR4D_T q;
	q.x = 0;
	q.y = 0;
	q.z = 0;
	q.w = 1;
	return q;
}

VECTOR4D_T quaternion_get_from_x(float rad) {
	VECTOR4D_T q;
	q.x = sin(rad * 0.5);
	q.y = 0;
	q.z = 0;
	q.w = cos(rad * 0.5);
	return q;
}

VECTOR4D_T quaternion_get_from_y(float rad) {
	VECTOR4D_T q;
	q.x = 0;
	q.y = sin(rad * 0.5);
	q.z = 0;
	q.w = cos(rad * 0.5);
	return q;
}

VECTOR4D_T quaternion_get_from_z(float rad) {
	VECTOR4D_T q;
	q.x = 0;
	q.y = 0;
	q.z = sin(rad * 0.5);
	q.w = cos(rad * 0.5);
	return q;
}

VECTOR4D_T quaternion_multiply(VECTOR4D_T q1, VECTOR4D_T q2) {
	VECTOR4D_T q;
	q.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
	q.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
	q.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
	q.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
	return q;
}

VECTOR4D_T quaternion_conjugate(VECTOR4D_T a) {
	VECTOR4D_T q;
	q.x = -a.x;
	q.y = -a.y;
	q.z = -a.z;
	q.w = a.w;
	return q;
}

VECTOR4D_T quaternion_normalize(VECTOR4D_T a) {
	float norm = sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
	if (norm == 0) { //fail safe
		norm = 1;
	}
	VECTOR4D_T q;
	q.x = a.x / norm;
	q.y = a.y / norm;
	q.z = a.z / norm;
	q.w = a.w / norm;
	return q;
}

static void twoaxisrot(float r11, float r12, float r21, float r31, float r32,
		float res[]) {
	res[0] = atan2(r11, r12);
	res[1] = acos(r21);
	res[2] = atan2(r31, r32);
}

static void threeaxisrot(float r11, float r12, float r21, float r31, float r32,
		float res[]) {
	res[0] = atan2(r31, r32);
	res[1] = asin(r21);
	res[2] = atan2(r11, r12);
}

void quaternion_get_euler(VECTOR4D_T q, float *_x, float *_y, float *_z,
		enum EULER_SEQUENCE seq) {
	float res[3];

	switch (rotSeq) {
	case EULER_SEQUENCE_ZYX:
		threeaxisrot(2 * (q.x * q.y + q.w * q.z),
				q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
				-2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x),
				q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z, res);
		break;

	case EULER_SEQUENCE_ZYZ:
		twoaxisrot(2 * (q.y * q.z - q.w * q.x), 2 * (q.x * q.z + q.w * q.y),
				q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
				2 * (q.y * q.z + q.w * q.x), -2 * (q.x * q.z - q.w * q.y), res);
		break;

	case EULER_SEQUENCE_ZXY:
		threeaxisrot(-2 * (q.x * q.y - q.w * q.z),
				q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
				2 * (q.y * q.z + q.w * q.x), -2 * (q.x * q.z - q.w * q.y),
				q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z, res);
		break;

	case EULER_SEQUENCE_ZXZ:
		twoaxisrot(2 * (q.x * q.z + q.w * q.y), -2 * (q.y * q.z - q.w * q.x),
				q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
				2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x), res);
		break;

	case EULER_SEQUENCE_YXZ:
		threeaxisrot(2 * (q.x * q.z + q.w * q.y),
				q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
				-2 * (q.y * q.z - q.w * q.x), 2 * (q.x * q.y + q.w * q.z),
				q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z, res);
		break;

	case EULER_SEQUENCE_YXY:
		twoaxisrot(2 * (q.x * q.y - q.w * q.z), 2 * (q.y * q.z + q.w * q.x),
				q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
				2 * (q.x * q.y + q.w * q.z), -2 * (q.y * q.z - q.w * q.x), res);
		break;

	case EULER_SEQUENCE_YZX:
		threeaxisrot(-2 * (q.x * q.z - q.w * q.y),
				q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
				2 * (q.x * q.y + q.w * q.z), -2 * (q.y * q.z - q.w * q.x),
				q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z, res);
		break;

	case EULER_SEQUENCE_YZY:
		twoaxisrot(2 * (q.y * q.z + q.w * q.x), -2 * (q.x * q.y - q.w * q.z),
				q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
				2 * (q.y * q.z - q.w * q.x), 2 * (q.x * q.y + q.w * q.z), res);
		break;

	case EULER_SEQUENCE_XYZ:
		threeaxisrot(-2 * (q.y * q.z - q.w * q.x),
				q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
				2 * (q.x * q.z + q.w * q.y), -2 * (q.x * q.y - q.w * q.z),
				q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z, res);
		break;

	case EULER_SEQUENCE_XYX:
		twoaxisrot(2 * (q.x * q.y + q.w * q.z), -2 * (q.x * q.z - q.w * q.y),
				q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
				2 * (q.x * q.y - q.w * q.z), 2 * (q.x * q.z + q.w * q.y), res);
		break;

	case EULER_SEQUENCE_XZY:
		threeaxisrot(2 * (q.y * q.z + q.w * q.x),
				q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
				-2 * (q.x * q.y - q.w * q.z), 2 * (q.x * q.z + q.w * q.y),
				q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z, res);
		break;

	case EULER_SEQUENCE_XZX:
		twoaxisrot(2 * (q.x * q.z - q.w * q.y), 2 * (q.x * q.y + q.w * q.z),
				q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
				2 * (q.x * q.z + q.w * q.y), -2 * (q.x * q.y - q.w * q.z), res);
		break;
	default:
		break;
	}

	if (_x) {
		*_x = res[0];
	}
	if (_y) {
		*_y = res[1];
	}
	if (_z) {
		*_z = res[2];
	}
}
