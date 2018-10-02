#ifndef _V4L2_HANDLER_H
#define _V4L2_HANDLER_H


typedef int (*PROCESS_IMAGE_CALLBACK)(const void *p, int size, void *user_data);
int handle_v4l2(const char *devicefile, int width, int height, int fps, PROCESS_IMAGE_CALLBACK _process_image, void *_user_data);

#endif
