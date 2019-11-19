#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <limits.h>
#include <sys/time.h>
#include <dlfcn.h>
#include <assert.h>

#include "picam360_image.h"

static int release(void *user_data) {
	PICAM360_IMAGE_T *image = (PICAM360_IMAGE_T*) user_data;
	for (int i = 0; i < 3; i++) {
		if (image->pixels[i] == NULL) {
			continue;
		}
		free(image->pixels[i]);
	}
	if (image->meta != NULL) {
		free(image->meta);
	}
	free(image);
}

int save_picam360_image(PICAM360_IMAGE_T **images, int num,
		size_t (*_write)(void*, void*, size_t), void *user_data) {
	for (int i = 0; i < num; i++) {
		PICAM360_IMAGE_T *image = images[i];
		char uuid_str[37]; //UUID_STR_LEN
		uuid_unparse_upper(image->uuid, uuid_str);

		char buff[4 * 1024];
		int len = 4;
		len += sprintf(buff + len, "<picam360:image ");
		len += sprintf(buff + len, "version=\"2.0\" ");
		len += sprintf(buff + len, "uuid=\"%s\" ", uuid_str);
		len += sprintf(buff + len, "timestamp=\"%ld,%ld\" ",
				image->timestamp.tv_sec, image->timestamp.tv_usec);
		len += sprintf(buff + len, "img_type=\"%.4s\" ", image->img_type);
		len += sprintf(buff + len, "meta_size=\"%d\" ", image->meta_size);
		len += sprintf(buff + len, "num_of_planes=\"%d\" ",
				image->num_of_planes);
		len += sprintf(buff + len, "width=\"%d,%d,%d\" ", image->width[0],
				image->width[1], image->width[2]);
		len += sprintf(buff + len, "stride=\"%d,%d,%d\" ", image->stride[0],
				image->stride[1], image->stride[2]);
		len += sprintf(buff + len, "height=\"%d,%d,%d\" ", image->height[0],
				image->height[1], image->height[2]);
		len += sprintf(buff + len, "/>");
		buff[0] = 'P';
		buff[1] = 'I';
		buff[2] = ((len - 4) & 0xff00) >> 8;
		buff[3] = ((len - 4) & 0x00ff) >> 0;
		int cur = 0;
		cur += _write(user_data, buff, len);
		cur += _write(user_data, image->meta, image->meta_size);
		for (int i = 0; i < 3; i++) {
			int size = image->stride[i] * image->height[i];
			if (size == 0) {
				continue;
			}
			cur += _write(user_data, image->pixels[i], size);
		}
	}
}

static size_t _write(void *user_data, void *data, size_t data_len) {
	return write((intptr_t) user_data, data, data_len);
}

int save_picam360_image_from_file(char *path, PICAM360_IMAGE_T **images,
		int num) {
	PICAM360_IMAGE_T *image = images[0];
	int fd = open(path, O_CREAT | O_WRONLY | O_TRUNC,
			S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);

	save_picam360_image(images, num, _write, (void*) (intptr_t) fd);

	close(fd);
}

int load_picam360_image(PICAM360_IMAGE_T **images_p, int *num_p,
		size_t (*_read)(void*, void*, size_t), void *user_data) {
	int num = *num_p;
	for (int i = 0; i < num; i++) {
		PICAM360_IMAGE_T *image = (PICAM360_IMAGE_T*) malloc(
				sizeof(PICAM360_IMAGE_T));
		memset(image, 0, sizeof(PICAM360_IMAGE_T));
		create_reference(&image->ref, release, image);

		char buff[4 * 1024];
		int cur = 0;
		cur += _read(user_data, buff, 4);
		if(cur != 4 || buff[0] != 'P' || buff[1] != 'I'){
			num = i;
			break;
		}
		int len = 0;
		len += (int) buff[2] << 8;
		len += (int) buff[3] << 0;
		cur += _read(user_data, buff + cur, len);
		buff[cur] = '\0';

		const int kMaxArgs = 32;
		int argc = 0;
		char *argv[kMaxArgs];
		char *p = strtok(buff + 4, " \n");
		while (p && argc < kMaxArgs - 1) {
			argv[argc++] = p;
			p = strtok(NULL, " \n");
		}
		for (int p = 0; p < argc; p++) {
			char *name = strtok(argv[p], "=");
			char *value = strtok(NULL, "=");
			if (strcmp(name, "uuid") == 0) {
				char uuid_str[37]; //UUID_STR_LEN
				sscanf(value, "\"%36s\"", uuid_str);
				uuid_parse(uuid_str, image->uuid);
			} else if (strcmp(name, "timestamp") == 0) {
				sscanf(value, "\"%ld,%ld\"", &image->timestamp.tv_sec,
						&image->timestamp.tv_usec);
			} else if (strcmp(name, "img_type") == 0) {
				char img_type[5] = { };
				sscanf(value, "\"%4s\"", img_type);
				memcpy(image->img_type, img_type, 4);
			} else if (strcmp(name, "meta_size") == 0) {
				sscanf(value, "\"%d\"", &image->meta_size);
			} else if (strcmp(name, "num_of_planes") == 0) {
				sscanf(value, "\"%d\"", &image->num_of_planes);
			} else if (strcmp(name, "width") == 0) {
				sscanf(value, "\"%d,%d,%d\"", &image->width[0],
						&image->width[1], &image->width[2]);
			} else if (strcmp(name, "stride") == 0) {
				sscanf(value, "\"%d,%d,%d\"", &image->stride[0],
						&image->stride[1], &image->stride[2]);
			} else if (strcmp(name, "height") == 0) {
				sscanf(value, "\"%d,%d,%d\"", &image->height[0],
						&image->height[1], &image->height[2]);
			}
		}
		if (image->meta_size > 0) {
			image->meta = (unsigned char*) malloc(image->meta_size + 1);
			cur += _read(user_data, image->meta, image->meta_size);
		}
		for (int i = 0; i < 3; i++) {
			int size = image->stride[i] * image->height[i];
			if (size == 0) {
				continue;
			}
			image->pixels[i] = (unsigned char*) malloc(size);
			cur += _read(user_data, image->pixels[i], size);
		}

		images_p[i] = image;
	}

	*num_p = num;

	return 0;
}

static size_t _read(void *user_data, void *data, size_t data_len) {
	return read((intptr_t) user_data, data, data_len);
}

int load_picam360_image_from_file(char *path, PICAM360_IMAGE_T **image_p,
		int *num_p) {
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		return -1;
	}

	load_picam360_image(image_p, num_p, _read, (void*) (intptr_t) fd);

	close(fd);

	return 0;
}
