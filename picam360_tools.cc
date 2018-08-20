/**
 * @file omxcv-test.cpp
 * @brief Simple testing application for omxcv.
 */
#include "picam360_tools.h"
#include "spline.hpp"
#include "omxcv.h"

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <thread>

#define TIMEDIFF(start) (duration_cast<microseconds>(steady_clock::now() - start).count())

using omxcv::OmxCv;
using omxcv::OmxCvJpeg;
using std::this_thread::sleep_for;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

//pre procedure difinition

//structure difinition

//global variables

void* StartRecord(const int width, const int height, const char *filename, int bitrate_kbps, int fps, RECORD_CALLBACK callback, void *user_data) {
	std::string _filename = filename;
	std::string extention = _filename.substr(_filename.find_last_of(".") + 1);
	if (extention == "h265") {
		h265_encoder *encoder = h265_create_encoder(width, height, bitrate_kbps, fps, callback, user_data);
		return (void*) encoder;
	} else {
		OmxCv *recorder = new OmxCv(filename, width, height, bitrate_kbps, fps, 1, callback, user_data);
		return (void*) recorder;
	}
}

int StopRecord(void *obj) {
	if (h265_is_encoder(obj)) {
		h265_delete_encoder((h265_encoder*) obj);
		return 0;
	} else {
		OmxCv *recorder = (OmxCv*) obj;
		if (recorder == NULL) {
			return -1;
		}
		delete recorder;
		recorder = NULL;
		return 0;
	}
}

int AddFrame(void *obj, const unsigned char *in_data, void *frame_data) {
	if (h265_is_encoder(obj)) {
		h265_add_frame((h265_encoder*) obj, in_data, frame_data);
		return 0;
	} else {
		OmxCv *recorder = (OmxCv*) obj;
		if (recorder == NULL) {
			return -1;
		}
		recorder->Encode(in_data, frame_data);
		return 0;
	}
}

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
