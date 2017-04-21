#pragma once

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
VECTOR4D_T quaternion_multiply(VECTOR4D_T a, VECTOR4D_T b);
