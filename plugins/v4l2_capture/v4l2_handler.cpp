/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see https://linuxtv.org/docs.php for more information
 */
//based on https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/capture.c.html
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>

#include <linux/videodev2.h>

#include "v4l2_handler.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

static struct timeval lg_init_time = {};
static struct timeval lg_start_time = {};

struct buffer {
	void *start;
	size_t length;
};

typedef struct _PARAMS_T {
	char dev_name[256];
	int width;
	int height;
	int fps;
	int fd;
	struct buffer *buffers;
	unsigned int n_buffers;
	int run;
	int num_of_frames;

	PROCESS_IMAGE_CALLBACK process_image;
	void *user_data;
} PARAMS_T;

static void errno_exit(const char *s) {
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg) {
	int r;

	do {
		r = ioctl(fh, request, arg);
	} while (-1 == r && EINTR == errno);

	return r;
}

static int read_frame(PARAMS_T *params) {

	struct v4l2_buffer buf;

	CLEAR(buf);

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(params->fd, VIDIOC_DQBUF, &buf)) {
		switch (errno) {
		case EAGAIN:
			return 0;

		case EIO:
			/* Could ignore EIO, see spec. */

			/* fall through */

		default:
			errno_exit("VIDIOC_DQBUF");
		}
	}

	struct timeval diff;
	struct timeval now;
	struct timeval timestamp;

    struct timespec  vsTime;
    clock_gettime(CLOCK_MONOTONIC, &vsTime);
    now.tv_sec = vsTime.tv_sec;
    now.tv_usec = vsTime.tv_nsec/1000.0;
	timersub(&now, &buf.timestamp, &diff);

	gettimeofday(&now, NULL);
	timersub(&now, &diff, &timestamp);

	assert(buf.index < params->n_buffers);

	params->run = params->process_image(params->buffers[buf.index].start, buf.bytesused, timestamp, params->user_data);
	params->num_of_frames++;

	if (-1 == xioctl(params->fd, VIDIOC_QBUF, &buf))
		errno_exit("VIDIOC_QBUF");

	return 1;
}

static void mainloop(PARAMS_T *params) {
	while (params->run) {
		for (;;) {
			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(params->fd, &fds);

			/* Timeout. */
			tv.tv_sec = 2;
			tv.tv_usec = 0;

			r = select(params->fd + 1, &fds, NULL, NULL, &tv);

			if (-1 == r) {
				if (EINTR == errno)
					continue;
				errno_exit("select");
			}

			if (0 == r) {
				fprintf(stderr, "select timeout\n");
				usleep(1000);
				continue;
			}

			if (read_frame(params))
				break;
			/* EAGAIN - continue select loop. */
		}
	}
}

static void stop_capturing(PARAMS_T *params) {
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(params->fd, VIDIOC_STREAMOFF, &type))
		errno_exit("VIDIOC_STREAMOFF");
}

static void start_capturing(PARAMS_T *params) {
	unsigned int i;
	enum v4l2_buf_type type;

	if(lg_init_time.tv_sec == 0){
		gettimeofday(&lg_init_time, NULL);
	}

	for (i = 0; i < params->n_buffers; ++i) {
		struct v4l2_buffer buf;

		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		if (-1 == xioctl(params->fd, VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");
	}
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	int wait = 0;
	struct timeval st;
	gettimeofday(&st, NULL);
	if(lg_start_time.tv_sec != 0){
		int span = (1000000 / params->fps);
		struct timeval diff;
		timersub(&st, &lg_init_time, &diff);
		wait = span - ((1000000 + diff.tv_usec) % span);
		usleep(wait);
		gettimeofday(&st, NULL);
	}
	if (-1 == xioctl(params->fd, VIDIOC_STREAMON, &type))
		errno_exit("VIDIOC_STREAMON");

	{
		struct timeval diff;
		timersub(&st, &lg_init_time, &diff);
		float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
		printf("started time : %f : %d\n", diff_sec, wait);
	}
	lg_start_time = st;
}

static void uninit_device(PARAMS_T *params) {
	unsigned int i;

	for (i = 0; i < params->n_buffers; ++i)
		if (-1 == munmap(params->buffers[i].start, params->buffers[i].length))
			errno_exit("munmap");

	free(params->buffers);
}

static void init_mmap(PARAMS_T *params) {
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count = params->n_buffers;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(params->fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
					"memory mapping\n", params->dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}

	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n", params->dev_name);
		exit(EXIT_FAILURE);
	}

	params->buffers = (struct buffer*)calloc(req.count, sizeof(*params->buffers));

	if (!params->buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (params->n_buffers = 0; params->n_buffers < req.count; ++params->n_buffers) {
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = params->n_buffers;

		if (-1 == xioctl(params->fd, VIDIOC_QUERYBUF, &buf))
			errno_exit("VIDIOC_QUERYBUF");

		params->buffers[params->n_buffers].length = buf.length;
		params->buffers[params->n_buffers].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, params->fd, buf.m.offset);

		if (MAP_FAILED == params->buffers[params->n_buffers].start)
			errno_exit("mmap");
	}
}

static void init_device(PARAMS_T *params) {
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	unsigned int min;

	if (-1 == xioctl(params->fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device\n", params->dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}
	printf("bus_info (%s): %s\n", params->dev_name, cap.bus_info);

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device\n", params->dev_name);
		exit(EXIT_FAILURE);
	}

	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		fprintf(stderr, "%s does not support streaming i/o\n", params->dev_name);
		exit(EXIT_FAILURE);
	}

	/* Select video input, video standard and tune here. */

	CLEAR(cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl(params->fd, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl(params->fd, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	} else {
		/* Errors ignored. */
	}

	CLEAR(fmt);

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = params->width;
	fmt.fmt.pix.height = params->height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	fmt.fmt.pix.field = V4L2_FIELD_NONE;

	if (-1 == xioctl(params->fd, VIDIOC_S_FMT, &fmt))
		errno_exit("VIDIOC_S_FMT");
	{
		struct v4l2_streamparm setfps;
		memset(&setfps, 0, sizeof(setfps));
		setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		setfps.parm.capture.timeperframe.numerator = 1;
		setfps.parm.capture.timeperframe.denominator = params->fps;
		if (-1 == xioctl(params->fd, VIDIOC_S_PARM, &setfps))
			errno_exit("VIDIOC_S_PARM");
	}

	/* Note VIDIOC_S_FMT may change width and height. */

	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;

	init_mmap(params);
}

static void close_device(PARAMS_T *params) {
	if (-1 == close(params->fd))
		errno_exit("close");

	params->fd = -1;
}

static int open_device(PARAMS_T *params) {
	struct stat st;

	if (-1 == stat(params->dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n", params->dev_name, errno, strerror(errno));
		return -1;
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", params->dev_name);
		return -1;
	}

	params->fd = open(params->dev_name, O_RDWR /* required */| O_NONBLOCK, 0);

	if (-1 == params->fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n", params->dev_name, errno, strerror(errno));
		return -1;
	}
	return 0;
}

int handle_v4l2(const char *devicefile, int n_buffers, int width, int height, int fps, PROCESS_IMAGE_CALLBACK _process_image, void *_user_data) {
	PARAMS_T params = { };
	params.fd = -1;
	params.run = 1;
	params.process_image = _process_image;
	params.user_data = _user_data;
	strncpy(params.dev_name, devicefile, sizeof(params.dev_name));
	params.width = width;
	params.height = height;
	params.fps = fps;
	params.n_buffers = n_buffers;

	if (open_device(&params) < 0) {
		return -1;
	}
	init_device(&params);
	start_capturing(&params);
	mainloop(&params);
	stop_capturing(&params);
	uninit_device(&params);
	close_device(&params);
	return 0;
}
