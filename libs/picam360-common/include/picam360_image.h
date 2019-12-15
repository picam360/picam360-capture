#pragma once

#include <pthread.h>
#include <stdbool.h>
#include <uuid/uuid.h>
#include "reference.h"

enum PICAM360_MEMORY_TYPE {
	PICAM360_MEMORY_TYPE_PROCESS,
	PICAM360_MEMORY_TYPE_KERNEL,
	PICAM360_MEMORY_TYPE_GPU,
	PICAM360_MEMORY_TYPE_EGL,
};

#define MAX_NUM_OF_PLANES 3
typedef struct _PICAM360_IMAGE_T {
	REFERENCE_H *ref;

	enum PICAM360_MEMORY_TYPE mem_type;

	uuid_t uuid;
	char img_type[4]; //"RGB"|"RGBA"|"I420"|"YUV"|"JPEG"|"H264"|"VP8"
	struct timeval timestamp;

	unsigned int meta_size;
	unsigned char *meta;

	unsigned int num_of_planes;
	unsigned int width[MAX_NUM_OF_PLANES];
	unsigned int height[MAX_NUM_OF_PLANES];
	unsigned int stride[MAX_NUM_OF_PLANES];
	unsigned char *pixels[MAX_NUM_OF_PLANES]; //size should be stride*height
	int id[MAX_NUM_OF_PLANES]; //PICAM360_MEMORY_TYPE_EGL
} PICAM360_IMAGE_T;

int save_picam360_image(PICAM360_IMAGE_T **images, int num,
		size_t (*_write)(void*, void*, size_t, int, int, char *), void *user_data);
int save_picam360_image_to_file(char *path, PICAM360_IMAGE_T **images,
		int num, bool pif_split);
int load_picam360_image(PICAM360_IMAGE_T **image_p, int *num_p,
		size_t (*_read)(void*, void*, size_t, int, int, char *), void *user_data);
int load_picam360_image_from_file(char *path, PICAM360_IMAGE_T **image_p,
		int *num_p);
