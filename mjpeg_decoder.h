#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "picam360_capture_plugin.h"

void init_mjpeg_decoder(PLUGIN_HOST_T *plugin_host, int cam_num, void **egl_images, int egl_image_num);
void deinit_mjpeg_decoder(int cam_num);
float mjpeg_decoder_get_fps(int cam_num);
int mjpeg_decoder_get_frameskip(int cam_num);
void mjpeg_decode(int cam_num, unsigned char *data, int data_len);
void mjpeg_decoder_switch_buffer(int cam_num);

#ifdef __cplusplus
}
#endif
