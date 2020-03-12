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
#include <map>
#if __linux
#include <sys/prctl.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "mrevent.h"
#include "tools.h"

#ifdef __cplusplus
}
#endif

#include "process_streamer.h"

using namespace std;

#define PLUGIN_NAME "process_streamer"
#define CAPTURE_NAME "process_streamer"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define BUFFER_NUM 4

typedef struct _process_streamer {
	VSTREAMER_T super;

	bool run;

	char def_name[64];
	char ipath[256];
	int iwidth;
	int iheight;

	char omode[32];
	char oimg_type[8];
	char opath[256];
	int owidth;
	int oheight;
	float ofps;
	int skipframe;

	unsigned int framecount;
	int frameskip;
	PICAM360_IMAGE_T frame_buffers[BUFFER_NUM];
	uint8_t xmp_buffers[BUFFER_NUM][RTP_MAXPAYLOADSIZE];
	MREVENT_T frame_ready;

	bool eof;
	pid_t pid;
	int pin_fd;
	int pout_fd;
	pthread_t streaming_thread;
} process_streamer;

static PLUGIN_HOST_T *lg_plugin_host = NULL;

#define R (0)
#define W (1)

static map<string, string> lg_defs;

static int get_frame_info_str(process_streamer *_this, char *buff,
		int *buff_sizse) {
	int len = 0;

	len += sprintf(buff + len, "<picam360:frame ");
	len += sprintf(buff + len, "mode=\"%s\" ", _this->omode);
	len += sprintf(buff + len, "/>");

	*buff_sizse = len;

	return 0;
}

static int progress_image(const void *p, int size, struct timeval timestamp,
		void *arg) {
	process_streamer *_this = (process_streamer*) arg;

	int cur = _this->framecount % BUFFER_NUM;

	PICAM360_IMAGE_T *frame = &_this->frame_buffers[cur];
	frame->timestamp = timestamp;
	frame->mem_type = PICAM360_MEMORY_TYPE_PROCESS;
	if (strcmp(_this->oimg_type, "RGB") == 0) {
		int size;
		frame->width[0] = _this->owidth;
		frame->stride[0] = _this->owidth * 3;
		frame->height[0] = _this->oheight;
		size = frame->stride[0] * frame->height[0];
		if (frame->pixels[0] == NULL) {
			frame->pixels[0] = (unsigned char*) malloc(size);
		}
		memcpy(frame->pixels[0], p, size);
		frame->num_of_planes = 1;
		memcpy(frame->img_type, "RGB\0", 4);
	} else if (strcmp(_this->oimg_type, "I420") == 0) {
		unsigned char *ptr = (unsigned char*) p;
		for (int i = 0; i < 3; i++) {
			int size;
			frame->width[i] = _this->owidth / (i==0?1:2);
			frame->stride[i] = _this->owidth / (i==0?1:2);
			frame->height[i] = _this->oheight / (i==0?1:2);
			size = frame->stride[i] * frame->height[i];
			if (frame->pixels[i] == NULL) {
				frame->pixels[i] = (unsigned char*) malloc(size);
			}
			memcpy(frame->pixels[i], ptr, size);
			ptr += size;
		}
		frame->num_of_planes = 3;
		memcpy(frame->img_type, "I420", 4);
	}

	frame->meta = _this->xmp_buffers[cur];
	get_frame_info_str(_this, (char*) frame->meta, (int*) &frame->meta_size);

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
	process_streamer *_this = (process_streamer*) arg;
	unsigned int data_len = 0;
	unsigned int buff_size = 64 * 1024;
	int frame_buffer_cur = 0;
	int frame_size;
	if (strcmp(_this->oimg_type, "RGB") == 0) {
		frame_size = _this->owidth * _this->oheight * 3;
	} else if (strcmp(_this->oimg_type, "I420") == 0) {
		frame_size = 3 * _this->owidth * _this->oheight / 2;
	} else {
		printf("something wrong!: %s\n", _this->oimg_type);
		return NULL;
	}
	unsigned char *frame_buffer = (unsigned char*) malloc(frame_size);
	unsigned char *data = (unsigned char*) malloc(buff_size);

	while (_this->run && (data_len = read(_this->pout_fd, data, buff_size)) > 0) {
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
	free(data);
	free(frame_buffer);

	int status;
	kill(_this->pid, SIGKILL);
	waitpid(_this->pid, &status, 0);

	printf("eof\n");
	_this->eof = true;
	_this->run = false;

	return NULL;
}

static void start(void *obj) {
	process_streamer *_this = (process_streamer*) obj;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}

	if (_this->owidth == 0) {
		_this->owidth = _this->iwidth;
	}
	if (_this->oheight == 0) {
		_this->oheight = _this->iheight;
	}

	map<string, string>::iterator itr = lg_defs.find(_this->def_name);
	if (itr == lg_defs.end()) {
		printf("no def_name! : %s\n", _this->def_name);
		return;
	}
	char oheight_str[64];
	sprintf(oheight_str, "%d", _this->oheight);

	char cmdline[1024];
	sprintf(cmdline, "%s", itr->second.c_str());
	strchg(cmdline, "${ipath}", _this->ipath);
	{
		char str[64];
		sprintf(str, "%d", _this->iwidth);
		strchg(cmdline, "${iwidth}", str);
	}
	strchg(cmdline, "${opath}", _this->opath);
	{
		char str[64];
		sprintf(str, "%d", _this->iheight);
		strchg(cmdline, "${iheight}", str);
	}
	{
		char str[64];
		sprintf(str, "%d", _this->owidth);
		strchg(cmdline, "${owidth}", str);
	}
	{
		char str[64];
		sprintf(str, "%d", _this->oheight);
		strchg(cmdline, "${oheight}", str);
	}

	const int MAX_ARGC = 128;
	int argc = 0;
	char *argv[MAX_ARGC];
	char *p = strtok(cmdline, " ");
	while (p && argc < MAX_ARGC - 1) {
		argv[argc++] = p;
		p = strtok(0, " ");
	}
	argv[argc++] = NULL;

	pid_t pid = 0;
	int pin_fd[2];
	int pout_fd[2];

	pipe(pin_fd);
	pipe(pout_fd);
	pid = fork();
	if (pid == 0) { // child process

		// connect parent
		dup2(pin_fd[R], STDIN_FILENO);
		dup2(pout_fd[W], STDOUT_FILENO);
		//dup2(pout_fd[W], STDERR_FILENO);

#if __linux
			//ask kernel to deliver SIGTERM in case the parent dies
			prctl(PR_SET_PDEATHSIG, SIGTERM);
#endif

		execvp(argv[0], argv);
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
	process_streamer *_this = (process_streamer*) obj;

	if (_this->run) {
		_this->run = false;
		pthread_join(_this->streaming_thread, NULL);
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static int set_param(void *obj, const char *param, const char *value_str) {
	process_streamer *_this = (process_streamer*) obj;
	if (strcmp(param, "def_name") == 0) {
		sscanf(value_str, "%64s", _this->def_name);
	} else if (strcmp(param, "omode") == 0) {
		sscanf(value_str, "%15s", _this->omode);
	} else if (strcmp(param, "oimg_type") == 0) {
		sscanf(value_str, "%4s", _this->oimg_type);
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
	} else if (strcmp(param, "ipath") == 0) {
		sscanf(value_str, "%255s", _this->ipath);
		int len = strlen(_this->ipath);
		if (len > 4 && strcasecmp(_this->ipath + len - 4, ".mp4") == 0) {
			char buff[1024];
			sprintf(buff,
					"MP4Box -info %s 2>&1 | awk '/Visual Info/ {print $3,$4}'",
					_this->ipath);
			FILE *fp = popen(buff, "r");
			if (fp == NULL) {
				printf("something wrong!: %s\n", buff);
			} else {
				size_t byte_count = fread(buff, 1, sizeof(buff) - 1, fp);
				buff[byte_count] = 0;
				int num = sscanf(buff, "width=%d height=%d", &_this->iwidth,
						&_this->iheight);
				if (num != 2) {
					printf("something wrong!: %s\n", buff);
				}
				pclose(fp);
			}
		}
	}

	return 0;
}

static int get_param(void *obj, const char *param, char *value, int size) {
	process_streamer *_this = (process_streamer*) obj;
	if (strcmp(param, "eof") == 0) {
		snprintf(value, size, "%s", _this->eof ? "true" : "false");
		return 0;
	}
	return 0;
}

static int get_image(void *obj, PICAM360_IMAGE_T **image_p, int *num_p,
		int wait_usec) {
	process_streamer *_this = (process_streamer*) obj;

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
	process_streamer *_this = (process_streamer*) obj;

	_this->super.stop(&_this->super);

	for (int i = 0; i < BUFFER_NUM; i++) {
		for (int j = 0; j < 3; j++) {
			if (_this->frame_buffers[i].pixels[j] != NULL) {
				free(_this->frame_buffers[i].pixels[j]);
				_this->frame_buffers[i].pixels[j] = NULL;
			}
		}
	}

	free(obj);
}

static void create_capture(void *user_data, VSTREAMER_T **out_capture) {
	VSTREAMER_T *capture = (VSTREAMER_T*) malloc(sizeof(process_streamer));
	memset(capture, 0, sizeof(process_streamer));
	strcpy(capture->name, CAPTURE_NAME);
	capture->release = release;
	capture->start = start;
	capture->stop = stop;
	capture->set_param = set_param;
	capture->get_param = get_param;
	capture->get_image = get_image;
	capture->user_data = capture;

	process_streamer *_private = (process_streamer*) capture;
	mrevent_init(&_private->frame_ready);
	strcpy(_private->omode, "EQUIRECTANGULAR");
	strcpy(_private->oimg_type, "I420");

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
	{ //defs
		json_t *obj = json_object_get(options, "defs");
		if (obj && json_is_array(obj)) {
			for (int i = 0; i < json_array_size(obj); i++) {
				json_t *i_obj = json_array_get(obj, i);

				json_t *name_obj = json_object_get(i_obj, "name");
				json_t *def_obj = json_object_get(i_obj, "def");
				if (name_obj == NULL || def_obj == NULL) {
					continue;
				}

				string name = json_string_value(name_obj);
				string def = json_string_value(def_obj);

				lg_defs.insert(map<string, string>::value_type(name, def));
			}
		}
	}
}

static void save_options(void *user_data, json_t *_options) {
	json_t *options = json_object();
	json_object_set_new(_options, PLUGIN_NAME, options);

	{
		json_t *obj = json_array();
		for (auto itr = lg_defs.begin(); itr != lg_defs.end(); itr++) {
			json_t *i_obj = json_object();
			json_object_set_new(i_obj, "name", json_string(itr->first.c_str()));
			json_object_set_new(i_obj, "def", json_string(itr->second.c_str()));
			json_array_append_new(obj, i_obj);
		}
		json_object_set_new(options, "defs", obj);
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
