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
#include <dlfcn.h>
#include <assert.h>
#include <sys/time.h>
#include <limits.h>
#include <list>
#if __linux
#include <sys/prctl.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "mrevent.h"

#ifdef __cplusplus
}
#endif

#include "ffmpeg_capture.h"

#define PLUGIN_NAME "ffmpeg_capture"
#define CAPTURE_NAME "ffmpeg_capture"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define BUFFER_NUM 4

typedef struct _ffmpeg_capture {
	VSTREAMER_T super;

	bool run;

	char ifilepath[256];
	char vstream_filepath[256];
	int cam_num;
	int owidth;
	int oheight;
	float ofps;
	int skipframe;

	unsigned int framecount;
	int frameskip;
	PICAM360_IMAGE_T frame_buffers[BUFFER_NUM];
	uint8_t xmp_buffers[BUFFER_NUM][RTP_MAXPAYLOADSIZE];
	MREVENT_T frame_ready;

	pid_t pid;
	int pin_fd;
	int pout_fd;
	pthread_t streaming_thread;
} ffmpeg_capture;

static PLUGIN_HOST_T *lg_plugin_host = NULL;
static char lg_devicefiles[MAX_CAM_NUM][256] = { };

#define R (0)
#define W (1)
static const int BYTES_PER_PIXEL = 3;

//rtp | uvc | file
static char lg_options_input_type[32] = { 'f', 'i', 'l', 'e' };
static char lg_options_input_codec[32] = { 'm', 'j', 'p', 'e', 'g' };

static int get_frame_info_str(char *buff, int *buff_sizse) {
	int len = 0;

	len += sprintf(buff + len, "<picam360:frame ");
	len += sprintf(buff + len, "mode=\"%s\" ", "EQUIRECTANGULAR");
	len += sprintf(buff + len, "/>");

	*buff_sizse = len;

	return 0;
}

static int progress_image(const void *p, int size, struct timeval timestamp,
		void *arg) {
	ffmpeg_capture *_this = (ffmpeg_capture*) arg;

	int cur = _this->framecount % BUFFER_NUM;

	PICAM360_IMAGE_T *frame = &_this->frame_buffers[cur];
	memset(frame, 0, sizeof(PICAM360_IMAGE_T));
	frame->timestamp = timestamp;
	frame->pixels[0] = (unsigned char*) p;
	frame->width[0] = _this->owidth;
	frame->stride[0] = _this->owidth * BYTES_PER_PIXEL;
	frame->height[0] = _this->oheight;
	memcpy(frame->img_type, "RGB\0", 4);
	frame->mem_type = PICAM360_MEMORY_TYPE_PROCESS;

	frame->meta = _this->xmp_buffers[cur];
	get_frame_info_str((char*) frame->meta, (int*) &frame->meta_size);

	_this->framecount++;
	mrevent_trigger(&_this->frame_ready);

	{
		static int count = 0;
		static float elapsed_sec = 0;
		float _elapsed_sec;
		struct timeval diff;

		struct timeval now;
		gettimeofday(&now, NULL);

		timersub(&now, &timestamp, &diff);
		_elapsed_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
		if ((count % 200) == 0) {
			printf("progress_image : %f\n", _elapsed_sec);
		}
		count++;
	}

	return _this->run ? 1 : 0;
}

static void* streaming_thread_fnc(void *arg) {
	ffmpeg_capture *_this = (ffmpeg_capture*) arg;
	unsigned int data_len = 0;
	unsigned int buff_size = 64 * 1024;
	int frame_buffer_cur = 0;
	int frame_size = _this->owidth * _this->oheight * BYTES_PER_PIXEL;
	unsigned char *frame_buffer = (unsigned char*) malloc(frame_size);
	unsigned char *data = (unsigned char*) malloc(buff_size);

	while ((data_len = read(_this->pout_fd, data, buff_size)) > 0) {
		//printf("%d ", data_len);
		for (int i = 0; i < data_len;) {
			if (frame_buffer_cur + (data_len - i) > frame_size) {
				memcpy(frame_buffer + frame_buffer_cur, data + i,
						(frame_size - frame_buffer_cur));
				i += (frame_size - frame_buffer_cur);
				frame_buffer_cur = frame_size;
			} else {
				memcpy(frame_buffer + frame_buffer_cur, data + i, data_len - i);
				frame_buffer_cur += data_len - i;
				i = data_len;
			}
			if (frame_buffer_cur == frame_size) {
				struct timeval timestamp;
				gettimeofday(&timestamp, NULL);
				progress_image(frame_buffer, frame_size, timestamp,
						(void*) _this);

				frame_buffer_cur = 0;
			}
		}
	}
	return NULL;
}

static void start(void *obj) {
	ffmpeg_capture *_this = (ffmpeg_capture*) obj;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}

	strncpy(_this->vstream_filepath, lg_devicefiles[_this->cam_num],
			sizeof(_this->vstream_filepath));

	pid_t pid = 0;
	int pin_fd[2];
	int pout_fd[2];
	//	_this->callback = callback;
	//	_this->user_data = user_data;
	//	_this->width = width;
	//	_this->height = height;

	pipe(pin_fd);
	pipe(pout_fd);
	pid = fork();
	if (pid == 0) { // child process
		const int MAX_ARGC = 128;
		int argc = 0;
		char *argv[MAX_ARGC];
		argv[argc++] = (char*) "ffmpeg";

		if (_this->ifilepath[0] == '/') {
			argv[argc++] = (char*) "-i";
			argv[argc++] = _this->ifilepath;
		} else if (strncmp(lg_options_input_type, "uvc",
				sizeof(lg_options_input_type)) == 0) {
			char size_str[16];

			sprintf(size_str, "%dx%d", _this->owidth, _this->oheight);
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
			argv[argc++] = (char*)"-f";
			argv[argc++] = (char*)"avfoundation";
			argv[argc++] = (char*)"-pix_fmt";
			argv[argc++] = (char*)"uyvy422";
			argv[argc++] = (char*)"-framerate";
			argv[argc++] = fps_str;
			argv[argc++] = (char*)"-video_size";
			argv[argc++] = size_str;
			argv[argc++] = (char*)"-i";
			argv[argc++] = (char*)"0";
#endif
#elif __linux
// linux
			argv[argc++] = (char*)"-pix_fmt";
			argv[argc++] = (char*)"uyvy422";
			//argv[argc++] = (char*)"-framerate";
			//argv[argc++] = fps_str;
			argv[argc++] = (char*)"-video_size";
			argv[argc++] = size_str;
			argv[argc++] = (char*)"-i";
			argv[argc++] = (char*)"/dev/video0";
#elif __unix // all unices not caught above
// Unix
#elif __posix
// POSIX
#endif
		} else {
			char size_str[16];
			char fps_str[16];

			sprintf(size_str, "%dx%d", _this->owidth, _this->oheight);
			sprintf(fps_str, "%d", 15);

			argv[argc++] = (char*) "-video_size";
			argv[argc++] = size_str;
			argv[argc++] = (char*) "-c:v";
			argv[argc++] = lg_options_input_codec;
			argv[argc++] = (char*) "-i";
			argv[argc++] = (char*) "pipe:0";
		}

		{ //output
			if (_this->ofps > 0) {
				char ofps_str[16];
				sprintf(ofps_str, "%f", _this->ofps);
				argv[argc++] = (char*) "-r";
				argv[argc++] = ofps_str;
			}
			argv[argc++] = (char*) "-pix_fmt";
			argv[argc++] = (char*) "rgb24";
			argv[argc++] = (char*) "-f";
			argv[argc++] = (char*) "rawvideo";
			argv[argc++] = (char*) "pipe:1";
		}
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

	_this->pid = pid;
	_this->pin_fd = pin_fd[W];
	_this->pout_fd = pout_fd[R];

	_this->run = true;

	pthread_create(&_this->streaming_thread, NULL, streaming_thread_fnc,
			(void*) _this);
}

static void stop(void *obj) {
	ffmpeg_capture *_this = (ffmpeg_capture*) obj;

	if (_this->run) {
		_this->run = false;
		pthread_join(_this->streaming_thread, NULL);
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static int set_param(void *obj, const char *param, const char *value_str) {
	ffmpeg_capture *_this = (ffmpeg_capture*) obj;
	if (strcmp(param, "cam_num") == 0) {
		int value = 0;
		sscanf(value_str, "%d", &value);
		_this->cam_num = MAX(0, MIN(value, MAX_CAM_NUM));
	} else if (strcmp(param, "owidth") == 0) {
		int value = 0;
		sscanf(value_str, "%d", &value);
		_this->owidth = MAX(64, MIN(value, 16384));	//8k:7680_4k:3840
	} else if (strcmp(param, "oheight") == 0) {
		int value = 0;
		sscanf(value_str, "%d", &value);
		_this->oheight = MAX(64, MIN(value, 16384));	//8k3d:7680_8k:3840
	} else if (strcmp(param, "ofps") == 0) {
		int value = 0;
		sscanf(value_str, "%d", &value);
		_this->ofps = MAX(0, MIN(value, 1000));
	} else if (strcmp(param, "ifilepath") == 0) {
		sscanf(value_str, "%256s", _this->ifilepath);
	}

	return 0;
}

static int get_param(void *obj, const char *param, char *value, int size) {
	return 0;
}

static int get_image(void *obj, PICAM360_IMAGE_T **image_p, int *num_p,
		int wait_usec) {
	ffmpeg_capture *_this = (ffmpeg_capture*) obj;

	int res = mrevent_wait(&_this->frame_ready, wait_usec);
	if (res != 0) {
		return -1;
	} else {
		mrevent_reset(&_this->frame_ready);
	}

	if (((_this->framecount - 1) % (_this->skipframe + 1)) != 0) {
		return -1;
	}

	int cur = (_this->framecount - 1) % BUFFER_NUM;
	PICAM360_IMAGE_T *frame = &_this->frame_buffers[cur];

	*image_p = frame;
	*num_p = 1;

	return 0;
}

static void release(void *obj) {
	ffmpeg_capture *_this = (ffmpeg_capture*) obj;

	_this->super.stop(&_this->super);

	free(obj);
}

static void create_capture(void *user_data, VSTREAMER_T **out_capture) {
	VSTREAMER_T *capture = (VSTREAMER_T*) malloc(sizeof(ffmpeg_capture));
	memset(capture, 0, sizeof(ffmpeg_capture));
	strcpy(capture->name, CAPTURE_NAME);
	capture->release = release;
	capture->start = start;
	capture->stop = stop;
	capture->set_param = set_param;
	capture->get_param = get_param;
	capture->get_image = get_image;
	capture->user_data = capture;

	ffmpeg_capture *_private = (ffmpeg_capture*) capture;
	mrevent_init(&_private->frame_ready);
	_private->owidth = 1024;
	_private->oheight = 512;
	_private->ofps = 0;

	if (out_capture) {
		*out_capture = capture;
	}
}

static int command_handler(void *user_data, const char *_buff) {
	int opt;
	int ret = 0;
	char buff[256] = { };
	strncpy(buff, _buff, sizeof(buff) - 1);
	char *cmd = strtok(buff, " \n");
	if (strncmp(cmd, "dummy", sizeof(buff)) == 0) {
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id,
		uint32_t event_id) {
}

static void init_options(void *user_data, json_t *_options) {
	PLUGIN_T *plugin = (PLUGIN_T*) user_data;
	json_t *options = json_object_get(_options, PLUGIN_NAME);
	if (options == NULL) {
		return;
	}
	for (int i = 0; i < MAX_CAM_NUM; i++) {
		char buff[256];
		sprintf(buff, "cam%d_devicefile", i);
		json_t *value = json_object_get(options, buff);
		if (value) {
			int len = json_string_length(value);
			if (len < sizeof(lg_devicefiles[i])) {
				strncpy(lg_devicefiles[i], json_string_value(value), len);
			}
		}
	}
}

static void save_options(void *user_data, json_t *_options) {
	json_t *options = json_object();
	json_object_set_new(_options, PLUGIN_NAME, options);

	for (int i = 0; i < MAX_CAM_NUM; i++) {
		char buff[256];
		if (lg_devicefiles[i][0] != 0) {
			sprintf(buff, "cam%d_devicefile", i);
			json_object_set_new(options, buff, json_string(lg_devicefiles[i]));
		}
	}
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
		VSTREAMER_FACTORY_T *factory = (VSTREAMER_FACTORY_T*) malloc(
				sizeof(VSTREAMER_FACTORY_T));
		memset(factory, 0, sizeof(VSTREAMER_FACTORY_T));
		strcpy(factory->name, CAPTURE_NAME);
		factory->release = release;
		factory->create_vstreamer = create_capture;

		lg_plugin_host->add_vstreamer_factory(factory);
	}
}
