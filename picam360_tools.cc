/**
 * @file omxcv-test.cpp
 * @brief Simple testing application for omxcv.
 */
#include "picam360_tools.h"
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

void* StartRecord(const int width, const int height, const char *filename,
		int bitrate_kbps) {
	OmxCv *recorder = new OmxCv(filename, width, height, bitrate_kbps);
	return (void*)recorder;
}

int StopRecord(void *obj) {
	OmxCv *recorder = (OmxCv*)obj;
	if (recorder == NULL) {
		return -1;
	}
	delete recorder;
	recorder = NULL;
	return 0;
}

int AddFrame(void *obj, const unsigned char *in_data) {
	OmxCv *recorder = (OmxCv*)obj;
	if (recorder == NULL) {
		return -1;
	}
	recorder->Encode(in_data);
	return 0;
}

int SaveJpeg(const unsigned char *in_data, const int width, const int height,
		const char *out_filename, int quality) {

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
