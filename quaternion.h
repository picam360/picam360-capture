#pragma once

typedef struct _QUATERNION {
	float x;
	float y;
	float z;
	float w;
} QUATERNION;

QUATERNION get_quaternion_from_x(float rad);
QUATERNION get_quaternion_from_y(float rad);
QUATERNION get_quaternion_from_z(float rad);
QUATERNION multiply_quaternion(QUATERNION a, QUATERNION b);
