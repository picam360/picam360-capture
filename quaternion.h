#pragma once

enum EULER_SEQUENCE {
	EULER_SEQUENCE_ZYX,
	EULER_SEQUENCE_ZYZ,
	EULER_SEQUENCE_ZXY,
	EULER_SEQUENCE_ZXZ,
	EULER_SEQUENCE_YXZ,
	EULER_SEQUENCE_YXY,
	EULER_SEQUENCE_YZX,
	EULER_SEQUENCE_YZY,
	EULER_SEQUENCE_XYZ,
	EULER_SEQUENCE_XYX,
	EULER_SEQUENCE_XZY,
	EULER_SEQUENCE_XZX
};

typedef union _VECTOR4D_T {
	struct {
		float x;
		float y;
		float z;
		float w;
	};
	float ary[4];
} VECTOR4D_T;

VECTOR4D_T quaternion_init();
VECTOR4D_T quaternion_get_from_x(float rad);
VECTOR4D_T quaternion_get_from_y(float rad);
VECTOR4D_T quaternion_get_from_z(float rad);
VECTOR4D_T quaternion_multiply(VECTOR4D_T a, VECTOR4D_T b); // Q = QbQa
VECTOR4D_T quaternion_conjugate(VECTOR4D_T q);
VECTOR4D_T quaternion_normalize(VECTOR4D_T a);
void quaternion_get_euler(VECTOR4D_T q, float *x, float *y, float *z, enum EULER_SEQUENCE seq);
