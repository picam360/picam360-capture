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
#include "tools.h"

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
		size_t (*_write)(void*, void*, size_t, int, int, char*),
		void *user_data) {
	for (int i = 0; i < num; i++) {
		PICAM360_IMAGE_T *image = images[i];

		char img_type[5] = { };
		memcpy(img_type, image->img_type, 4);

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
		cur += _write(user_data, buff, len, i, 0, NULL);
		cur += _write(user_data, image->meta, image->meta_size, i, 0, NULL);
		for (int j = 0; j < 3; j++) {
			int size = image->stride[j] * image->height[j];
			if (size == 0) {
				continue;
			}
			cur += _write(user_data, image->pixels[i], size, i, j, img_type);
		}
	}
}

static size_t _write(void *user_data, void *data, size_t data_len, int img_num,
		int plane_num, char *img_type) {
	void **params = user_data;
	if (img_type == NULL || params[1] == NULL) {
		return write((intptr_t) params[0], data, data_len);
	} else {
		char path[512] = { };
		strncpy(path, params[1], sizeof(path));
		snprintf(path, sizeof(path) - 1, "%s.%d.%d.%s", (char*) params[1],
				img_num, plane_num, img_type);

		int ret;
		ret = mkdir_path(path, 0775);
		if (ret < 0) {
			printf("can not make directory : %s\n", path);
			return -1;
		}

		int fd = open(path, O_CREAT | O_WRONLY | O_TRUNC,
				S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		if (fd < 0) {
			printf("can not make file : %s\n", path);
			return -1;
		}
		write(fd, data, data_len);
		close(fd);

		return 0;
	}
}

int save_picam360_image_to_file(char *path, PICAM360_IMAGE_T **images, int num,
		bool pif_split) {

	int ret;
	ret = mkdir_path(path, 0775);
	if (ret < 0) {
		printf("can not make directory : %s\n", path);
		return -1;
	}

	void *params[2] = { };

	int fd = open(path, O_CREAT | O_WRONLY | O_TRUNC,
			S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
	if (fd < 0) {
		printf("can not make file : %s\n", path);
		return -1;
	}
	params[0] = (void*) (intptr_t) fd;
	if (pif_split) {
		params[1] = (void*) path;
	}

	save_picam360_image(images, num, _write, params);

	close(fd);
}

int load_picam360_image(PICAM360_IMAGE_T **images_p, int *num_p,
		size_t (*_read)(void*, void*, size_t, int, int, char*), void *user_data) {
	int num = *num_p;
	for (int i = 0; i < num; i++) {
		PICAM360_IMAGE_T *image = (PICAM360_IMAGE_T*) malloc(
				sizeof(PICAM360_IMAGE_T));
		memset(image, 0, sizeof(PICAM360_IMAGE_T));
		create_reference(&image->ref, release, image);

		char buff[4 * 1024];
		int cur = 0;
		cur += _read(user_data, buff, 4, i, 0, NULL);
		if (cur != 4 || buff[0] != 'P' || buff[1] != 'I') {
			num = i;
			break;
		}
		int len = 0;
		len += (int) buff[2] << 8;
		len += (int) buff[3] << 0;
		cur += _read(user_data, buff + cur, len, i, 0, NULL);
		buff[cur] = '\0';

		const int kMaxArgs = 32;
		int argc = 0;
		char *argv[kMaxArgs];
		char *p = strtok(buff + 4, " \n");
		while (p && argc < kMaxArgs - 1) {
			argv[argc++] = p;
			p = strtok(NULL, " \n");
		}
		char img_type[5] = { };
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
			cur += _read(user_data, image->meta, image->meta_size, i, 0, NULL);
		}
		for (int j = 0; j < 3; j++) {
			int size = image->stride[j] * image->height[j];
			if (size == 0) {
				continue;
			}
			image->pixels[j] = (unsigned char*) malloc(size);
			cur += _read(user_data, image->pixels[j], size, i, j, img_type);
		}

		images_p[i] = image;
	}

	*num_p = num;

	return 0;
}

static size_t _read(void *user_data, void *data, size_t data_len, int img_num,
		int plane_num, char *img_type) {
	void **params = user_data;
	if (img_type == NULL) {
		return read((intptr_t) params[0], data, data_len);
	} else {
		char path[512] = { };
		strncpy(path, params[1], sizeof(path));
		snprintf(path, sizeof(path) - 1, "%s.%d.%d.%s", (char*) params[1],
				img_num, plane_num, img_type);

		int fd = open(path, O_RDONLY);
		if (fd < 0) {
			return read((intptr_t) params[0], data, data_len);
		} else {
			int n = read(fd, data, data_len);
			close(fd);
			return n;
		}
	}
}

int load_picam360_image_from_file(char *path, PICAM360_IMAGE_T **image_p,
		int *num_p) {
	void *params[2] = { };

	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		printf("can not read file : %s\n", path);
		return -1;
	}
	params[0] = (void*) (intptr_t) fd;
	params[1] = (void*) path;

	load_picam360_image(image_p, num_p, _read, params);

	close(fd);

	return 0;
}
