#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "h265_encoder.h"
#define CLASS_NAME "h265"

typedef struct _h265_encoder {
	char class_name[8];
	//nal
	bool in_nal;
	uint32_t nal_len;
	uint8_t nal_type;
	uint8_t *nal_buff;
	uint32_t nal_pos;

	pthread_mutex_t frame_data_queue_mutex;
	void *frame_data_queue[16];
	int frame_data_queue_cur;
	int frame_data_queue_last_cur;

	pid_t pid;
	int pin_fd;
	int pout_fd;
	int width;
	int height;
	pthread_t pout_thread;
	H265_STREAM_CALLBACK callback;
	void *user_data;
} h265_encoder;

bool h265_is_encoder(void *obj) {
	return (strcmp((char*) obj, CLASS_NAME) == 0);
}

static void *get_frame_data(h265_encoder *_this) {
	void *frame_data = NULL;
	if ((_this->nal_type >= 0 && _this->nal_type <= 9) || (_this->nal_type >= 16 && _this->nal_type <= 21)) {
		pthread_mutex_lock(&_this->frame_data_queue_mutex);
		if (_this->frame_data_queue_cur >= _this->frame_data_queue_last_cur) {
			printf("something wrong!");
		} else {
			frame_data = _this->frame_data_queue[_this->frame_data_queue_cur % 16];
			_this->frame_data_queue[_this->frame_data_queue_cur % 16] = NULL;
			_this->frame_data_queue_cur++;
		}
		pthread_mutex_unlock(&_this->frame_data_queue_mutex);
	}
	return frame_data;
}

static void *pout_thread_func(void* arg) {
	unsigned int data_len = 0;
	unsigned int buff_size = 64 * 1024;
	unsigned char *data = malloc(buff_size);
	h265_encoder *_this = (h265_encoder*) arg;
	while ((data_len = read(_this->pout_fd, data, buff_size)) > 0) {
		for (int i = 0; i < data_len;) {
			if (!_this->in_nal) {
				_this->in_nal = true;
				_this->nal_len = data[i] << 24 | data[i + 1] << 16 | data[i + 2] << 8 | data[i + 3];
				_this->nal_len += 4; //start code
				_this->nal_type = (data[i + 4] & 0x7e) >> 1;
				if (_this->nal_len > 1024 * 1024) {
					printf("something wrong in h264 stream at %d\n", i);
					_this->in_nal = false;

					break;
				}
				if (i + _this->nal_len <= data_len) {
					void *frame_data = get_frame_data(_this);
					_this->callback(data + i, _this->nal_len, frame_data, _this->user_data);
					i += _this->nal_len;
					_this->in_nal = false;
				} else {
					_this->nal_pos = 0;
					_this->nal_buff = (uint8_t*) malloc(_this->nal_len);
				}
			} else {
				int rest = _this->nal_len - _this->nal_pos;
				if (i + rest <= data_len) {
					memcpy(_this->nal_buff + _this->nal_pos, data + i, rest);

					void *frame_data = get_frame_data(_this);
					_this->callback(_this->nal_buff, _this->nal_len, frame_data, _this->user_data);

					free(_this->nal_buff);
					_this->nal_buff = NULL;

					i += rest;
					_this->in_nal = false;
				} else {
					int len = data_len - i;
					memcpy(_this->nal_buff + _this->nal_pos, data + i, len);
					_this->nal_pos += len;
					i += len;
				}
			}
		}
	}
	return NULL;
}

#define R (0)
#define W (1)
h265_encoder *h265_create_encoder(const int width, const int height, int bitrate_kbps, int fps, H265_STREAM_CALLBACK callback, void *user_data) {
	pid_t pid = 0;
	int pin_fd[2];
	int pout_fd[2];
	char size_str[16];
	char fps_str[16];
	char kbps_str[16];

	sprintf(size_str, "%dx%d", width, height);
	sprintf(fps_str, "%d", fps);
	sprintf(kbps_str, "%dk", bitrate_kbps);

	h265_encoder *_this = malloc(sizeof(h265_encoder));
	memset(_this, 0, sizeof(h265_encoder));
	sprintf(_this->class_name, CLASS_NAME);
	_this->callback = callback;
	_this->user_data = user_data;
	_this->width = width;
	_this->height = height;

	pipe(pin_fd);
	pipe(pout_fd);
	pid = fork();
	if (pid == 0) {
		// Child
		dup2(pin_fd[R], STDIN_FILENO);
		dup2(pout_fd[W], STDOUT_FILENO);
		//dup2(pout_fd[W], STDERR_FILENO);

		//ask kernel to deliver SIGTERM in case the parent dies
		prctl(PR_SET_PDEATHSIG, SIGTERM);

		execlp("ffmpeg", "ffmpeg", "-f", "rawvideo", "-framerate", fps_str, "-video_size", size_str, "-pix_fmt", "rgb24", "-i", "pipe:0", "-c:v", "libx265", "-x265-params",
				"annexb=0:repeat-headers=1", "-pix_fmt", "yuv420p", "-preset", "ultrafast", "-tune", "zerolatency", "-vb", kbps_str, "-f", "rawvideo", "pipe:1", (char*) NULL);
		// Nothing below _this line should be executed by child process. If so,
		// it means that the execl function wasn't successfull, so lets exit:
		exit(1);
	}
	// The code below will be executed only by parent. You can write and read
	// from the child using pipefd descriptors, and you can send signals to
	// the process using its pid by kill() function. If the child process will
	// exit unexpectedly, the parent process will obtain SIGCHLD signal that
	// can be handled (e.g. you can respawn the child process).

	//close unused pipe ends
	close(pin_fd[R]);
	close(pout_fd[W]);

	pthread_mutex_init(&_this->frame_data_queue_mutex, 0);
	pthread_create(&_this->pout_thread, NULL, pout_thread_func, (void*) _this);

	_this->pid = pid;
	_this->pin_fd = pin_fd[W];
	_this->pout_fd = pout_fd[R];
	return _this;
}
void h265_delete_encoder(h265_encoder *_this) {
	int status;
	kill(_this->pid, SIGKILL); //send SIGKILL signal to the child process
	waitpid(_this->pid, &status, 0);
}
void h265_add_frame(h265_encoder *_this, const unsigned char *in_data, void *frame_data) {
	pthread_mutex_lock(&_this->frame_data_queue_mutex);
	if (_this->frame_data_queue_last_cur >= _this->frame_data_queue_cur + 16) {
		printf("buffer is too small!");
	} else {
		_this->frame_data_queue[_this->frame_data_queue_last_cur % 16] = frame_data;
		_this->frame_data_queue_last_cur++;
	}
	pthread_mutex_unlock(&_this->frame_data_queue_mutex);

	write(_this->pin_fd, in_data, _this->width * _this->height * 3);
}

#ifdef H265_TEST
static void stream_callback(unsigned char *data, unsigned int data_len, void *frame_data, void *user_data) {
	int out_fd = (int) user_data;
	write(out_fd, data, data_len);
}
int main(int argc, char *argv[]) {
	int in_fd = -1;
	int out_fd = -1;
	int size = 0;
	int frame_size = 512 * 512 * 3;
	unsigned char *frame_buffer = NULL;
	h265_encoder *encoder = NULL;

	if (argc < 3) {
		fprintf(stderr, "argc:%d", argc);
		return -1;
	}

	in_fd = open(argv[1], O_RDONLY);
	if (in_fd < 0) {
		fprintf(stderr, "in_fd:%s", argv[1]);
		return -1;
	}
	out_fd = open(argv[2], O_CREAT | O_WRONLY | O_TRUNC);
	if (out_fd < 0) {
		fprintf(stderr, "out_fd:%s", argv[2]);
		return -1;
	}

	frame_buffer = (unsigned char *) malloc(frame_size);
	encoder = h265_create_encoder(512, 512, 256, 5, stream_callback, (void*) out_fd);
	while ((size = read(in_fd, frame_buffer, frame_size)) == frame_size) {
		h265_add_frame(encoder, frame_buffer, NULL);
		usleep(200 * 1000);
	}
	usleep(1000 * 1000);
	h265_delete_encoder(encoder);
	close(in_fd);
	close(out_fd);
}
#endif
