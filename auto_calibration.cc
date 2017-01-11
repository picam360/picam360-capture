#include "auto_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <stdbool.h>

#ifdef __cplusplus
}
#endif

#include <opencv/cv.h>
#include <opencv/highgui.h>

static void auto_calibration(PICAM360CAPTURE_T *state, FRAME_T *frame);
static void deinit(PICAM360CAPTURE_T *state, FRAME_T *frame);

void set_auto_calibration(FRAME_T *frame) {
	frame->after_processed_callback = auto_calibration;
	frame->befor_deleted_callback = deinit;
}
bool is_auto_calibration(FRAME_T *frame) {
	return frame->after_processed_callback == auto_calibration;
}

static void auto_calibration(PICAM360CAPTURE_T *state, FRAME_T *frame) {
	int margin = 32;
	int width = frame->width + 2 * margin;
	int height = frame->height + 2 * margin;
	int offset_x = frame->width * state->options.cam_offset_x[0];
	int offset_y = frame->height * state->options.cam_offset_y[0];

	if (frame->custom_data == NULL) { // first call
		frame->custom_data = (void*) cvCreateImage(cvSize(width, height),
				IPL_DEPTH_8U, 1);

		//reset
		state->options.cam_offset_x[0] = 0.0;
		state->options.cam_offset_y[0] = 0.0;

		return;
	}

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contour = NULL;
	IplImage *img = (IplImage*) frame->custom_data;
	IplImage *image_bin = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	//binalize
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			uint8_t val = 0;

			uint32_t _x = x - margin - offset_x;
			uint32_t _y = y - margin - offset_y;
			if (_x >= 0 && _x < frame->width && _y >= 0 && _y < frame->height) {
				uint8_t ch1 = (frame->img_buff + frame->width * 3 * _y)[_x * 3 + 0];
				uint8_t ch2 = (frame->img_buff + frame->width * 3 * _y)[_x * 3 + 1];
				uint8_t ch3 = (frame->img_buff + frame->width * 3 * _y)[_x * 3 + 2];
				val = MAX(ch1, MAX(ch2, ch3));
			}

			val = (val >= 32) ? 255 : 0;

			unsigned char _val = (img->imageData + img->widthStep * y)[x];

			(img->imageData + img->widthStep * y)[x] = MAX(_val, val);
		}
	}
	cvCopy(img, image_bin);

	//find countor
	cvErode(image_bin, image_bin, 0, 5);
	cvDilate(image_bin, image_bin, 0, 5);
	//cvSaveImage("debug.jpeg", image_bin);
	cvFindContours(image_bin, storage, &contour, sizeof(CvContour),
			CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

	// loop over all contours
	{
		CvBox2D box = { };
		//CvPoint2D32f box_point[4];
		CvBox2D tmp;
		double max_val = INT_MIN;
		CvSeq *cp = contour;
		while (cp != NULL) {
			double size;
			tmp = cvMinAreaRect2(cp, storage);
			size = tmp.size.width * tmp.size.height;
			if (size > max_val) {
				box = tmp;
				max_val = size;
			}
			cp = cp->h_next;
		}
		//printf("%lf,%lf\n", box.center.x, box.center.y);
		if (box.size.width > (float) frame->width * 0.8
				&& box.size.height > (float) frame->height * 0.8) {
			state->options.cam_offset_x[0] = box.center.x / width - 0.5;
			state->options.cam_offset_y[0] = box.center.y / height - 0.5;
		}
	}

	//Release
	if (storage != NULL) {
		cvReleaseMemStorage(&storage);
	}
	if (image_bin != NULL) {
		cvReleaseImage(&image_bin);
	}

}

static void deinit(PICAM360CAPTURE_T *state, FRAME_T *frame) {
	if (frame->custom_data == NULL) {
		IplImage *img = (IplImage*) frame->custom_data;
		cvReleaseImage(&img);
		frame->custom_data = NULL;
	}
}
