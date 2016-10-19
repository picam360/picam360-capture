/**
 * @file omxcv-test.cpp
 * @brief Simple testing application for omxcv.
 */
#include "picam360_tools.h"
#include "omxcv.h"
#include "gl_transform.h"
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

using namespace openblw;

//pre procedure difinition

//structure difinition

//global variables
static int TEXURE_WIDTH = 0;
static int TEXURE_HEIGHT = 0;
static int EQUIRECTANGULAR_WIDTH = 0;
static int EQUIRECTANGULAR_HEIGHT = 0;
static int X_DEG = 0;
static int Y_DEG = 0;
static int Z_DEG = 0;
static int JPEG_QUALITY = 90;

static OmxCvJpeg *encoder = NULL;
static GLTransform *transformer = NULL;
static OmxCv *recorder = NULL;

int TransformToEquirectangular(int texture_width, int texture_height,
		int equirectangular_width, int equirectangular_height,
		const unsigned char *in_data, unsigned char *out_data) {

	if (texture_width != TEXURE_WIDTH || texture_height != TEXURE_HEIGHT
			|| equirectangular_width != EQUIRECTANGULAR_WIDTH
			|| equirectangular_height != EQUIRECTANGULAR_HEIGHT) {
		TEXURE_WIDTH = texture_width;
		TEXURE_HEIGHT = texture_height;
		EQUIRECTANGULAR_WIDTH = equirectangular_width;
		EQUIRECTANGULAR_HEIGHT = equirectangular_height;

		if (encoder != NULL) {
			delete encoder;
			encoder = NULL;
		}
		if (transformer != NULL) {
			delete transformer;
			transformer = NULL;
		}

		transformer = new GLTransform(EQUIRECTANGULAR_WIDTH,
				EQUIRECTANGULAR_HEIGHT, TEXURE_WIDTH, TEXURE_HEIGHT);
	}
	transformer->SetRotation(X_DEG, Y_DEG, Z_DEG);
	transformer->Transform(in_data, out_data);

	return 0;
}

int SetRotation(float x_deg, float y_deg, float z_deg) {
	X_DEG = x_deg;
	Y_DEG = y_deg;
	Z_DEG = z_deg;
}

int StartRecord(const char *filename, int bitrate_kbps) {
	recorder = new OmxCv(filename, EQUIRECTANGULAR_WIDTH,
			EQUIRECTANGULAR_HEIGHT, bitrate_kbps);
	return 0;
}

int StopRecord() {
	if (recorder == NULL)
		return -1;
	delete recorder;
	recorder = NULL;
	return 0;
}

int AddFrame( const unsigned char *in_data) {
	if (recorder == NULL)
		return -1;

	recorder->Encode(in_data);
}

int SaveJpeg(const unsigned char *in_data, const char *out_filename, int quality) {
	if(JPEG_QUALITY != quality) {
		JPEG_QUALITY = quality;
		if (encoder != NULL) {
			delete encoder;
			encoder = NULL;
		}
	}
	if (encoder == NULL) {
		encoder = new OmxCvJpeg(EQUIRECTANGULAR_WIDTH, EQUIRECTANGULAR_HEIGHT, JPEG_QUALITY);
	}
	if (out_filename != NULL) {
		if (encoder->Encode(out_filename, in_data)) {
		} else {
			perror("error on jpeg encode");
			return -1;
		}
	}

	return 0;
}
