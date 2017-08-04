#ifndef _AUTO_CALIBRATION_H
#define _AUTO_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "picam360_capture.h"

void set_auto_calibration(FRAME_T *frame);
bool is_auto_calibration(FRAME_T *frame);

#ifdef __cplusplus
}
#endif

#endif
