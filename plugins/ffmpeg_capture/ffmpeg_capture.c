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

#ifdef USE_GLES
#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#else
#include <GL/glew.h>
#include <GLFW/glfw3.h>
//#include "GL/gl.h"
//#include "GL/glut.h"
//#include "GL/glext.h"
#endif

#include "ffmpeg_capture.h"

#define PLUGIN_NAME "ffmpeg_capture"
#define CAPTURE_NAME "ffmpeg"

static PLUGIN_HOST_T *lg_plugin_host = NULL;
static int lg_width = 2048;
static int lg_height = 1536;
static int lg_fps = 10;
static const int BYTES_PER_PIXEL = 3;

//rtp or uvc
static char lg_options_input_type[32] = { 'u', 'v', 'c' };
static char lg_options_input_codec[32] = { 'm', 'j', 'p', 'e', 'g' };

typedef struct _ffmpeg_capture {
	CAPTURE_T super;

	int cam_num;
	GLFWwindow *glfw_window;
	GLuint *cam_texture;
	uint32_t frame_num;

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
	ENCODER_STREAM_CALLBACK callback;
	void *user_data;
} ffmpeg_capture;

static void *pout_thread_func(void* arg) {
	unsigned int data_len = 0;
	unsigned int buff_size = 64 * 1024;
	int frame_buffer_cur = 0;
	int frame_size = lg_width * lg_height * BYTES_PER_PIXEL;
	unsigned char *frame_buffer = malloc(frame_size);
	unsigned char *data = malloc(buff_size);
	ffmpeg_capture *_this = (ffmpeg_capture*) arg;

	glfwMakeContextCurrent(_this->glfw_window);

	while ((data_len = read(_this->pout_fd, data, buff_size)) > 0) {
		//printf("%d ", data_len);
		for (int i = 0; i < data_len;) {
			if (frame_buffer_cur + (data_len - i) > frame_size) {
				memcpy(frame_buffer + frame_buffer_cur, data + i, (frame_size - frame_buffer_cur));
				i += (frame_size - frame_buffer_cur);
				frame_buffer_cur = frame_size;
			} else {
				memcpy(frame_buffer + frame_buffer_cur, data + i, data_len - i);
				frame_buffer_cur += data_len - i;
				i = data_len;
			}
			if (frame_buffer_cur == frame_size) {
				lg_plugin_host->lock_texture();
				{
					glBindTexture(GL_TEXTURE_2D, _this->cam_texture[0]);
					glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, lg_width, lg_height, 0, GL_RGB, GL_UNSIGNED_BYTE, frame_buffer);
				}
				lg_plugin_host->unlock_texture();

				frame_buffer_cur = 0;
				_this->frame_num++;
//				if (_this->frame_num % 1000 == 0) {
//					printf("frame num:%d\n", _this->frame_num);
//				}
			}
		}
	}
	return NULL;
}

#define R (0)
#define W (1)
static void start(void *obj, int cam_num, void *display, void *context, int egl_image_num) {
	ffmpeg_capture *_this = (ffmpeg_capture*) obj;
	pid_t pid = 0;
	int pin_fd[2];
	int pout_fd[2];
//	_this->callback = callback;
//	_this->user_data = user_data;
//	_this->width = width;
//	_this->height = height;
	_this->cam_num = cam_num;
	_this->glfw_window = (GLFWwindow*) display;
	_this->cam_texture = (GLuint*) cam_texture;

	pipe(pin_fd);
	pipe(pout_fd);
	pid = fork();
	if (pid == 0) {
		char size_str[16];
		char fps_str[16];

		const int MAX_ARGC = 128;
		int argc = 0;
		char **argv = malloc(MAX_ARGC * sizeof(char*));
		argv[argc++] = "ffmpeg";

		sprintf(size_str, "%dx%d", lg_width, lg_height);
		sprintf(fps_str, "%d", 15);

		if (strncmp(lg_options_input_type, "uvc", sizeof(lg_options_input_type)) == 0) {
#ifdef _WIN64
//define something for Windows (64-bit)
#elif _WIN32
//define something for Windows (32-bit)
#elif __APPLE__
#include "TargetConditionals.h"
#if TARGET_OS_IPHONE && TARGET_IPHONE_SIMULATOR
			// define something for simulator
#elif TARGET_OS_IPHONE
			// define something for iphone
#else
#define TARGET_OS_OSX 1
			// define something for OSX
			argv[argc++] = "-f";
			argv[argc++] = "avfoundation";
			argv[argc++] = "-pix_fmt";
			argv[argc++] = "uyvy422";
			argv[argc++] = "-framerate";
			argv[argc++] = fps_str;
			argv[argc++] = "-video_size";
			argv[argc++] = size_str;
			argv[argc++] = "-i";
			argv[argc++] = "0";
#endif
#elif __linux
// linux
			argv[argc++] = "-pix_fmt";
			argv[argc++] = "uyvy422";
			argv[argc++] = "-framerate";
			argv[argc++] = fps_str;
			argv[argc++] = "-video_size";
			argv[argc++] = size_str;
			argv[argc++] = "-i";
			argv[argc++] = "/dev/video0";
#elif __unix // all unices not caught above
// Unix
#elif __posix
// POSIX
#endif
		} else {
			argv[argc++] = "-video_size";
			argv[argc++] = size_str;
			argv[argc++] = "-c:v";
			argv[argc++] = lg_options_input_codec;
			argv[argc++] = "-i";
			argv[argc++] = "pipe:0";
		}

		//output
		argv[argc++] = "-r";
		argv[argc++] = fps_str;
		argv[argc++] = "-pix_fmt";
		argv[argc++] = "rgb24";
		argv[argc++] = "-f";
		argv[argc++] = "rawvideo";
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
	ffmpeg_capture *_this = (ffmpeg_capture*) obj;
	int status;
	kill(_this->pid, SIGKILL); //send SIGKILL signal to the child process
	waitpid(_this->pid, &status, 0);
	free(obj);
}

static float get_fps(void *user_data) {
	return 0;
}

static void create_capture(void *user_data, CAPTURE_T **out_capture) {
	CAPTURE_T *capture = (CAPTURE_T*) malloc(sizeof(ffmpeg_capture));
	memset(capture, 0, sizeof(ffmpeg_capture));
	strcpy(capture->name, CAPTURE_NAME);
	capture->release = release;
	capture->init = init;
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
