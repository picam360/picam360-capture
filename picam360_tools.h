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

typedef void (*RECORD_CALLBACK)(unsigned char *data, unsigned int data_len, void *frame_data, void *user_data);

void *StartRecord(const int width, const int height, const char *filename, int bitrate_kbps, int fps, RECORD_CALLBACK callback, void *user_data);
int StopRecord(void *);
int AddFrame(void *, const unsigned char *in_data, void *frame_data);
int SaveJpeg(const unsigned char *in_data, const int width, const int height, const char *out_filename, int quality);

void get_cubic_spline(int num_of_points, float *x_ary, float *y_ary, int num_of_points2, float *x_ary2, float *out_y_ary2);

POINT_T QuadraticBezPoint(POINT_T p0, POINT_T p1, POINT_T p2, float d) ;

#ifdef __cplusplus
}
#endif

#endif
