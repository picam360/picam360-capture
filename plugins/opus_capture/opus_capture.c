#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#if __linux
#include <sys/prctl.h>
#endif

#include "opus_capture.h"

#define PT_AUDIO_BASE 120

#define PLUGIN_NAME "opus_capture"
#define CAPTURE_NAME "opus_capture"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

typedef struct _opus_capture {
	CAPTURE_T super;

	uint32_t frame_num;

	pthread_mutex_t frame_data_queue_mutex;
	void *frame_data_queue[16];
	int frame_data_queue_cur;
	int frame_data_queue_last_cur;

	pid_t pid;
	int pin_fd;
	int pout_fd;
	pthread_t pout_thread;
	ENCODER_STREAM_CALLBACK callback;
	void *user_data;
} opus_capture;

static void *pout_thread_func(void* arg) {
	const char sc[5] = { 'O', 'g', 'g', 'S', '\0' };
	int marker = 0;
	unsigned int data_len = 0;
	unsigned int buff_size = 64 * 1024;
	int frame_buffer_cur = 0;
	unsigned int frame_size = 64 * 1024;
	unsigned int frame_len = 0;
	unsigned char *frame_buffer = malloc(frame_size);
	unsigned int data_cur = 0;
	unsigned char *data = malloc(buff_size);
	opus_capture *_this = (opus_capture*) arg;

	unsigned int header_len = 0;
	unsigned int comment_len = 0;
	unsigned char *header_buffer = NULL;
	unsigned char *comment_buffer = NULL;

	while ((data_len = read(_this->pout_fd, data, buff_size)) > 0) {
		//printf("%d ", data_len);
#define OPUS_BOUNDARY
#ifdef OPUS_BOUNDARY
		data_cur = 0;
		for (int i = 0; i < data_len; i++) {
			if (marker == 0 && data[i] == 'O') {
				marker++;
			} else if (marker == 1 && data[i] == 'g') {
				marker++;
			} else if (marker == 2 && data[i] == 'g') {
				marker++;
			} else if (marker == 3 && data[i] == 'S') {
				if (frame_len != 0) { // avoid garbage data
					int len = (i + 1) - 4 - data_cur;
					if (len > 0) {
						if (frame_len + len > frame_size) { // resize buffer
							char *new_buff;
							frame_size += len;
							new_buff = malloc(frame_size);
							memcpy(new_buff, frame_buffer, frame_len);
							free(frame_buffer);
							frame_buffer = new_buff;
						}
						memcpy(frame_buffer + frame_len, data + data_cur, len);
					}
					frame_len += len;
					memcpy(frame_buffer, sc, 4); //start code
					if (header_buffer == NULL && strncmp(frame_buffer + 28, "OpusHead", strlen("OpusHead")) == 0) {
						printf("OpusHead found\n", data_len);
						header_len = frame_len;
						header_buffer = malloc(header_len);
						memcpy(header_buffer, frame_buffer, header_len);
					} else if (comment_buffer == NULL && strncmp(frame_buffer + 28, "OpusTags", strlen("OpusTags")) == 0) {
						printf("OpusTags found\n", data_len);
						comment_len = frame_len;
						comment_buffer = malloc(comment_len);
						memcpy(comment_buffer, frame_buffer, comment_len);
					}else if (header_buffer && comment_buffer){
						rtp_sendpacket(lg_plugin_host->get_rtp(), header_buffer, header_len, PT_AUDIO_BASE + 0);
						rtp_sendpacket(lg_plugin_host->get_rtp(), comment_buffer, comment_len, PT_AUDIO_BASE + 0);
						rtp_sendpacket(lg_plugin_host->get_rtp(), frame_buffer, frame_len, PT_AUDIO_BASE + 0);
						rtp_flush(lg_plugin_host->get_rtp());
					}
				}

				frame_len = 4;
				data_cur = i + 1;
				marker = 0;
			} else {
				marker = 0;
			}
		}
		if (frame_len != 0) { // avoid garbage data
			int len = data_len - data_cur;
			if (len > 0) {
				if (frame_len + len > frame_size) { // resize buffer
					char *new_buff;
					frame_size += len;
					new_buff = malloc(frame_size);
					memcpy(new_buff, frame_buffer, frame_len);
					free(frame_buffer);
					frame_buffer = new_buff;
				}
				memcpy(frame_buffer + frame_len, data + data_cur, len);
			}
			frame_len += len;
		}
#else
		rtp_sendpacket(lg_plugin_host->get_rtp(), data, data_len, PT_AUDIO_BASE + 0);
#endif
	}
	return NULL;
}

#define R (0)
#define W (1)
static void start(void *obj, int cam_num, void *display, void *context, void *cam_texture, int egl_image_num) {
	opus_capture *_this = (opus_capture*) obj;
	pid_t pid = 0;
	int pin_fd[2];
	int pout_fd[2];

	pipe(pin_fd);
	pipe(pout_fd);
	pid = fork();
	if (pid == 0) {
		const int MAX_ARGC = 128;
		int argc = 0;
		char **argv = malloc(MAX_ARGC * sizeof(char*));
		argv[argc++] = "ffmpeg";
		argv[argc++] = "-v";
		argv[argc++] = "quiet";
		argv[argc++] = "-f";
		argv[argc++] = "alsa";
		argv[argc++] = "-i";
		argv[argc++] = "hw:1,0";
		argv[argc++] = "-strict";
		argv[argc++] = "-2";
		argv[argc++] = "-b:a";
		argv[argc++] = "64k";
		argv[argc++] = "-opus_delay";
		argv[argc++] = "20";

		//output
		argv[argc++] = "-f";
		argv[argc++] = "opus";
		argv[argc++] = "pipe:1";
		argv[argc++] = NULL;

		// Child
		dup2(pin_fd[R], STDIN_FILENO);
		dup2(pout_fd[W], STDOUT_FILENO);
		//dup2(pout_fd[W], STDERR_FILENO);

#if __linux
		//ask kernel to deliver SIGTERM in case the parent dies
		prctl(PR_SET_PDEATHSIG, SIGTERM);
#endif

		execvp("ffmpeg", argv);
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
}
static void release(void *obj) {
	opus_capture *_this = (opus_capture*) obj;
	int status;
	kill(_this->pid, SIGKILL); //send SIGKILL signal to the child process
	waitpid(_this->pid, &status, 0);
	free(obj);
}

static float get_fps(void *user_data) {
	return 0;
}

static void create_capture(void *user_data, CAPTURE_T **out_capture) {
	CAPTURE_T *capture = (CAPTURE_T*) malloc(sizeof(opus_capture));
	memset(capture, 0, sizeof(opus_capture));
	strcpy(capture->name, CAPTURE_NAME);
	capture->release = release;
	capture->start = start;
	capture->get_fps = get_fps;
	capture->user_data = capture;

	if (out_capture) {
		*out_capture = capture;
	}
}

static int command_handler(void *user_data, const char *_buff) {
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
}

static void init_options(void *user_data, json_t *options) {
}

static void save_options(void *user_data, json_t *options) {
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	lg_plugin_host = plugin_host;

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(PLUGIN_T));
		memset(plugin, 0, sizeof(PLUGIN_T));
		strcpy(plugin->name, PLUGIN_NAME);
		plugin->release = release;
		plugin->command_handler = command_handler;
		plugin->event_handler = event_handler;
		plugin->init_options = init_options;
		plugin->save_options = save_options;
		plugin->get_info = NULL;
		plugin->user_data = plugin;

		*_plugin = plugin;
	}
	{
		CAPTURE_FACTORY_T *capture_factory = (CAPTURE_FACTORY_T*) malloc(sizeof(CAPTURE_FACTORY_T));
		memset(capture_factory, 0, sizeof(CAPTURE_FACTORY_T));
		strcpy(capture_factory->name, CAPTURE_NAME);
		capture_factory->release = release;
		capture_factory->create_capture = create_capture;

		lg_plugin_host->add_capture_factory(capture_factory);
	}
}
