#ifndef PICAM360_TOOLS_H
#define PICAM360_TOOLS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "h265_encoder.h"

typedef struct _POINT_T {
	float x;
	float y;
} POINT_T;

void get_cubic_spline(int num_of_points, float *x_ary, float *y_ary, int num_of_points2, float *x_ary2, float *out_y_ary2);

POINT_T QuadraticBezPoint(POINT_T p0, POINT_T p1, POINT_T p2, float d) ;

#ifdef __cplusplus
}
#endif

#endif
