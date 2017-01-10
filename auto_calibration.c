#include "auto_calibration.h"

#include <opencv/core.h>

void auto_calibration(PICAM360CAPTURE_T *state, FRAME_T *frame) {

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contour = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq),
			sizeof(CvPoint), storage);

	IplImage *image_bin = cvCreateImage(cvSize(frame->width, frame->height),
			IPL_DEPTH_8U, 1);

	int margin = 32;
	int width = frame->width + 2 * margin;
	int height = frame->height + 2 * margin;

	//binalize
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			unsigned char val = 0;

			int _x = x - margin;
			int _y = y - margin;
			if (_x >= 0 && _y >= 0) {
				val = (frame->img_buff + frame->width * 3 * _y)[_x * 3];
			}

			val = (val >= 32) ? 255 : 0;

			(image_bin->imageData + image_bin->widthStep * y)[x] = val;
		}
	}

	//find countor
	cvDilate(image_bin, image_bin, 0, 1);
	cvErode(image_bin, image_bin, 0, 1);
	cvFindContours(image_bin, storage, &contour, sizeof(CvContour),
			CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

	// loop over all contours
	{
		CvBox2D box = { };
		CvPoint2D32f box_point[4];
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
		printf("%d,%d\n", box.center.x, box.center.y)
	}

	//Release
	if (storage != NULL) {
		cvReleaseMemStorage(&storage);
	}
	if (image_bin != NULL) {
		cvReleaseImage(&image_bin);
	}

}
