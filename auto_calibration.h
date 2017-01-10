#ifndef _AUTO_CALIBRATION_H
#define _AUTO_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "picam360_capture.h"

void auto_calibration(PICAM360CAPTURE_T *state, FRAME_T *frame);

#ifdef __cplusplus
}
#endif

#endif
