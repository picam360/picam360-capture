/**
 * @file omxcv-test.cpp
 * @brief Simple testing application for omxcv.
 */
#include "picam360_tools.h"
#include "spline.hpp"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <thread>

//pre procedure difinition

//structure difinition

//global variables

#if(0)
int SaveJpeg(const unsigned char *in_data, const int width, const int height, const char *out_filename, int quality) {

	OmxCvJpeg encoder(width, height, quality);
	if (out_filename != NULL) {
		if (encoder.Encode(out_filename, in_data)) {
		} else {
			perror("error on jpeg encode");
			return -1;
		}
	}

	return 0;
}
#endif

void get_cubic_spline(int num_of_points, float *x_ary, float *y_ary, int num_of_points2, float *x_ary2, float *out_y_ary2) {
	std::vector<double> X, Y;

	for (int i = 0; i < num_of_points; i++) {
		X.push_back(x_ary[i]);
		Y.push_back(y_ary[i]);
	}

	tk::spline s;
	s.set_points(X, Y);

	for (int i = 0; i < num_of_points2; i++) {
		out_y_ary2[i] = (float) s(x_ary2[i]);
	}
}

POINT_T QuadraticBezPoint(POINT_T p0, POINT_T p1, POINT_T p2, float d) {

	POINT_T o = { 0, 0 };

	float v = (1 - d) * (1 - d);
	o.x += v * p0.x;
	o.y += v * p0.y;

	v = 2 * d * (1 - d);
	o.x += v * p1.x;
	o.y += v * p1.y;

	v = d * d;
	o.x += v * p2.x;
	o.y += v * p2.y;

	return o;
}
