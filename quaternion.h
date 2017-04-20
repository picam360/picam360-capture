#pragma once

typedef union _QUATERNION_T {
	struct {
		float x;
		float y;
		float z;
		float w;
	};
	float ary[4];
} QUATERNION_T;

QUATERNION_T quaternion_init();
QUATERNION_T quaternion_get_from_x(float rad);
QUATERNION_T quaternion_get_from_y(float rad);
QUATERNION_T quaternion_get_from_z(float rad);
QUATERNION_T quaternion_multiply(QUATERNION_T a, QUATERNION_T b);
