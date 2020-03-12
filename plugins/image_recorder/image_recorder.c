#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <limits.h>
#include <dirent.h>
#include <dlfcn.h>
#include <assert.h>

#include "image_recorder.h"
#include "mrevent.h"
#include "tools.h"

#define PLUGIN_NAME "image_recorder"
#define STREAMER_NAME "image_recorder"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static PLUGIN_HOST_T *lg_plugin_host = NULL;

enum RECORDER_MODE {
	RECORDER_MODE_IDLE, RECORDER_MODE_RECORD, RECORDER_MODE_PLAY
};

#define BUFFER_NUM 4
typedef struct _image_recorder_private {
	VSTREAMER_T super;

	bool run;
	pthread_t streaming_thread;

	int framecount;
	PICAM360_IMAGE_T *frame_buffers[BUFFER_NUM][MAX_CAM_NUM + 1];
	MREVENT_T frame_ready;

	char tag[128];
	char base_path[257];
	char meta_path[257];
	enum RECORDER_MODE mode;
	bool repeat;
	float fps;

	bool eof;
	bool pif_split; //split pif
	int record_framecount;
	int limit_record_framecount;
	int skipframe;
	int play_framecount;
	int play_framecount_offset;
	struct timeval last_timestamp;
	struct timeval base_timestamp;
} image_recorder_private;

static image_recorder_private *lg_menu_image_recorder = NULL;

static char lg_record_path[512] = "Videos";
static enum RECORDER_MODE lg_recorder_mode = RECORDER_MODE_IDLE;

static void* streaming_thread_func(void *obj) {
	image_recorder_private *_this = (image_recorder_private*) obj;

	while (_this->run) {
		if (_this->super.pre_streamer == NULL) {
			usleep(100 * 1000);
			continue;
		}
		int ret;
		int num = MAX_CAM_NUM;
		PICAM360_IMAGE_T *images[MAX_CAM_NUM];
		ret = _this->super.pre_streamer->get_image(_this->super.pre_streamer,
				images, &num, 100 * 1000);
		if (ret != 0) {
			continue;
		}
		bool new_frame = true;
		int cur = _this->framecount % BUFFER_NUM;
		for (int i = 0; i < num; i++) {
			if (_this->frame_buffers[cur][i] != NULL
					&& _this->frame_buffers[cur][i]->ref) {
				_this->frame_buffers[cur][i]->ref->release(
						_this->frame_buffers[cur][i]->ref);
				_this->frame_buffers[cur][i] = NULL;
			}
			_this->frame_buffers[cur][i] = images[i];
		}
		_this->frame_buffers[cur][num] = NULL;
		_this->framecount++;
		mrevent_trigger(&_this->frame_ready);

		if (_this->mode == RECORDER_MODE_RECORD
				&& _this->framecount % (_this->skipframe + 1) == 0) {
			char path[512];
			++_this->record_framecount; //start from 1
			snprintf(path, sizeof(path) - 1, "%s/%d.pif", _this->base_path,
					_this->record_framecount);
			ret = save_picam360_image_to_file(path, images, num,
					_this->pif_split);
			if (_this->record_framecount == _this->limit_record_framecount) {
				_this->mode = RECORDER_MODE_IDLE;
			}
		}
	}

	return NULL;
}

static void start(void *user_data) {
	image_recorder_private *_this = (image_recorder_private*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}

	_this->run = true;
	pthread_create(&_this->streaming_thread, NULL, streaming_thread_func,
			(void*) _this);
}

static void stop(void *user_data) {
	image_recorder_private *_this = (image_recorder_private*) user_data;

	if (_this->run) {
		_this->run = false;
		pthread_join(_this->streaming_thread, NULL);
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static size_t read_file(const char *file, unsigned char **data) {
	if (data == NULL) {
		return 0;
	}

	FILE *fp = fopen(file, "rb");
	size_t length = 0;

	if (fp) {
		fseek(fp, 0, SEEK_END);
		length = ftell(fp);
		fseek(fp, 0, SEEK_SET);

		*data = (unsigned char*) malloc(length + 1);
		length = fread(*data, 1, length, fp);
		(*data)[length] = '\0';

		fclose(fp);
	}

	return length;
}

static int image_release(void *user_data) {
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

static int get_image(void *obj, PICAM360_IMAGE_T **images_p, int *num_p,
		int wait_usec) {
	image_recorder_private *_this = (image_recorder_private*) obj;

	if (_this->mode == RECORDER_MODE_PLAY) {
		bool no_more_frame = false;
		char path[512];
		if (strcasecmpr(_this->base_path, ".jpeg") == 0
				|| strcasecmpr(_this->base_path, ".jpg") == 0) {
			sprintf(path, _this->base_path,
					_this->play_framecount - _this->play_framecount_offset + 1);
			unsigned char *pixels = NULL;
			int len = read_file(path, &pixels);
			if (pixels == NULL) {
				no_more_frame = true;
			} else {
				PICAM360_IMAGE_T *frame = (PICAM360_IMAGE_T*) malloc(
						sizeof(PICAM360_IMAGE_T));
				memset(frame, 0, sizeof(PICAM360_IMAGE_T));
				create_reference(&frame->ref, image_release, frame);
				//frame->timestamp = timestamp;
				frame->mem_type = PICAM360_MEMORY_TYPE_PROCESS;
				frame->width[0] = len;
				frame->stride[0] = len;
				frame->height[0] = 1;
				frame->pixels[0] = pixels;
				frame->num_of_planes = 1;
				memcpy(frame->img_type, "JPEG", 4);
				frame->meta_size = read_file(_this->meta_path, &frame->meta);

				images_p[0] = frame;
				*num_p = 1;
			}
			if (_this->fps <= 0) {
				_this->fps = 1;
			}
		} else {
			sprintf(path, "%s/%d.pif", _this->base_path,
					_this->play_framecount - _this->play_framecount_offset + 1);
			strchg(path, "${tag}", _this->tag);

			int ret = load_picam360_image_from_file(path, images_p, num_p);
			no_more_frame = (ret != 0);
		}
		if (no_more_frame) {
			if (_this->play_framecount == 0) {
				printf("not found %s\n", path);
				sleep(1);
			} else if (_this->repeat) {
				printf("repeat\n");
				_this->play_framecount_offset = _this->play_framecount;
				sleep(1); //wait 1sec
			} else {
				printf("eof\n");
				_this->eof = true;
				_this->mode = RECORDER_MODE_IDLE;
			}
			return -1;
		} else {
			_this->play_framecount++;
		}
		struct timeval diff;
		float elapsed_sec;

		if (_this->fps <= 0) {
			if (_this->play_framecount >= 2) {
				timersub(&images_p[0]->timestamp, &_this->last_timestamp,
						&diff);
				elapsed_sec = (float) diff.tv_sec
						+ (float) diff.tv_usec / 1000000;
				_this->fps = 1.0 / elapsed_sec;
			}
		} else {
			struct timeval now;
			gettimeofday(&now, NULL);

			float elapsed_sec_target = (float) _this->play_framecount
					/ _this->fps;

			timersub(&now, &_this->base_timestamp, &diff);
			elapsed_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
			elapsed_sec_target -= elapsed_sec;
			if (elapsed_sec_target > 60) {
				elapsed_sec_target = 60;
				printf("something wrong : %s\n", __FILE__);
			}
			if (elapsed_sec_target > 0) {
				usleep(elapsed_sec_target * 1000000);
			}
		}
		_this->last_timestamp = images_p[0]->timestamp;
		gettimeofday(&images_p[0]->timestamp, NULL);
		return 0;
	} else if (_this->super.pre_streamer == NULL) {
		usleep(wait_usec);
		return -1;
	} else {
		int res = mrevent_wait(&_this->frame_ready, wait_usec);
		if (res != 0) {
			return -1;
		} else {
			mrevent_reset(&_this->frame_ready);
		}
		int cur = (_this->framecount - 1) % BUFFER_NUM;

		int num = 0;
		for (int i = 0; num < *num_p && i < MAX_CAM_NUM; i++) {
			PICAM360_IMAGE_T *image = _this->frame_buffers[cur][i];
			if (image == NULL) {
				break;
			}
			if (image->ref) {
				image->ref->addref(image->ref);
			}
			images_p[num] = image;

			num++;
		}
		*num_p = num;
		return 0;
	}
	return -1;
}

static int set_param(void *obj, const char *param, const char *value_str) {
	image_recorder_private *_this = (image_recorder_private*) obj;
	if (strcmp(param, "uuid") == 0) {
		char uuid_str[37] = { };
		sscanf(value_str, "%36s", uuid_str);
		uuid_parse(uuid_str, _this->super.uuid);
		return 0;
	} else if (strcmp(param, "pif_split") == 0) {
		_this->pif_split = (value_str[0] == '1' || value_str[0] == 't'
				|| value_str[0] == 'T');
		return 0;
	} else if (strcmp(param, "mode") == 0) {
		_this->mode = RECORDER_MODE_IDLE;
		_this->record_framecount = 0;
		_this->play_framecount = 0;
		_this->play_framecount_offset = 0;
		_this->eof = false;
		gettimeofday(&_this->base_timestamp, NULL);
		if (strcmp(value_str, "IDLE") == 0) {
			//do nothing
		} else if (strcmp(value_str, "RECORD") == 0) {
			_this->mode = RECORDER_MODE_RECORD;
		} else if (strcmp(value_str, "PLAY") == 0) {
			_this->mode = RECORDER_MODE_PLAY;
		}
		return 0;
	} else if (strcmp(param, "base_path") == 0) {
		int len = snprintf(_this->base_path, sizeof(_this->base_path) - 1, "%s",
				value_str);
		if (_this->base_path[len - 1] == '/') {
			_this->base_path[len - 1] = '\0';
			len--;
		}
		return 0;
	} else if (strcmp(param, "meta_path") == 0) {
		int len = snprintf(_this->meta_path, sizeof(_this->meta_path) - 1, "%s",
				value_str);
		return 0;
	} else if (strcmp(param, "tag") == 0) {
		int len = snprintf(_this->tag, sizeof(_this->tag) - 1, "%s", value_str);
		if (strcmp(value_str, "menu") == 0) {
			lg_menu_image_recorder = _this;
		}
		return 0;
	} else if (strcmp(param, "repeat") == 0) {
		_this->repeat = (value_str[0] == '1' || value_str[0] == 't'
				|| value_str[0] == 'T');
		return 0;
	} else if (strcmp(param, "fps") == 0) {
		sscanf(value_str, "%f", &_this->fps);
		return 0;
	} else if (strcmp(param, "skipframe") == 0) {
		sscanf(value_str, "%d", &_this->skipframe);
		return 0;
	} else if (strcmp(param, "limit_record_framecount") == 0) {
		sscanf(value_str, "%d", &_this->limit_record_framecount);
		return 0;
	}

	return -1;
}

static int get_param(void *obj, const char *param, char *value, int size) {
	image_recorder_private *_this = (image_recorder_private*) obj;
	if (strcmp(param, "eof") == 0) {
		snprintf(value, size, "%s", _this->eof ? "true" : "false");
		return 0;
	}
	return -1;
}

static void release(void *obj) {
	image_recorder_private *_this = (image_recorder_private*) obj;

	if (_this == lg_menu_image_recorder) {
		lg_menu_image_recorder = NULL;
	}
	if (_this->run) {
		_this->super.stop(&_this->super);
	}
	for (int cur = 0; cur < BUFFER_NUM; cur++) {
		for (int i = 0; i < MAX_CAM_NUM; i++) {
			if (_this->frame_buffers[cur][i] != NULL
					&& _this->frame_buffers[cur][i]->ref) {
				_this->frame_buffers[cur][i]->ref->release(
						_this->frame_buffers[cur][i]->ref);
				_this->frame_buffers[cur][i] = NULL;
			}
			_this->frame_buffers[cur][i] = NULL;
		}
	}

	free(obj);
}

static void create_vstreamer(void *user_data, VSTREAMER_T **output_streamer) {
	VSTREAMER_T *streamer = (VSTREAMER_T*) malloc(
			sizeof(image_recorder_private));
	memset(streamer, 0, sizeof(image_recorder_private));
	strcpy(streamer->name, STREAMER_NAME);
	streamer->release = release;
	streamer->start = start;
	streamer->stop = stop;
	streamer->set_param = set_param;
	streamer->get_param = get_param;
	streamer->get_image = get_image;
	streamer->user_data = streamer;

	image_recorder_private *_private = (image_recorder_private*) streamer;
	mrevent_init(&_private->frame_ready);

	if (output_streamer) {
		*output_streamer = streamer;
	}
}

static int command_handler(void *user_data, const char *_buff) {
	int opt;
	int ret = 0;
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd = strtok(buff, " \n");
	if (strncmp(cmd, "stop", sizeof(buff)) == 0) {
		image_recorder_private *streamer = lg_menu_image_recorder;
		if (streamer != NULL) {
			streamer->super.set_param(&streamer->super, "mode", "IDLE");
		}
	} else if (strncmp(cmd, "record", sizeof(buff)) == 0) {
		char *param = strtok(NULL, "\n");
		if (param != NULL) {
			char *path = param;
			image_recorder_private *streamer = lg_menu_image_recorder;
			if (streamer != NULL) {
				streamer->super.set_param(&streamer->super, "base_path", path);
				streamer->super.set_param(&streamer->super, "mode", "RECORD");
			}
		}
	} else if (strncmp(cmd, "play", sizeof(buff)) == 0) {
		char *param = strtok(NULL, "\n");
		if (param != NULL) {
			char *path = param;
			image_recorder_private *streamer = lg_menu_image_recorder;
			if (streamer != NULL) {
				streamer->super.set_param(&streamer->super, "base_path", path);
				streamer->super.set_param(&streamer->super, "mode", "PLAY");
				streamer->super.set_param(&streamer->super, "fps", "0");
			}
		}
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
}

static void record_menu_record_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED:
		break;
	case MENU_EVENT_DEACTIVATED:
		break;
	case MENU_EVENT_SELECTED:
		menu->selected = false;
		if (lg_recorder_mode == RECORDER_MODE_RECORD) { //stop record
			lg_recorder_mode = RECORDER_MODE_IDLE;

			char cmd[512];
			sprintf(cmd, PLUGIN_NAME ".stop");
			lg_plugin_host->send_command(cmd);

			menu->marked = false;
			menu->selected = false;
			printf("stop recording\n");
		} else if (lg_recorder_mode == RECORDER_MODE_IDLE) { //start record
			if (lg_menu_image_recorder == NULL
					|| lg_menu_image_recorder->super.pre_streamer == NULL) {
				menu->selected = false;
				break;
			}
			lg_recorder_mode = RECORDER_MODE_RECORD;

			time_t t;
			time(&t);
			struct tm *lt = localtime(&t);
			char name[128] = { };
			snprintf(name, sizeof(name) - 1, "%4d%2d%2d_%2d%2d%2d",
					lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
					lt->tm_hour, lt->tm_min, lt->tm_sec);

			char filepath[256];
			sprintf(filepath, "%s/%s", lg_record_path, name);

			char cmd[512];
			sprintf(cmd, PLUGIN_NAME ".record %s", filepath);
			lg_plugin_host->send_command(cmd);

			printf("start recording %s\n", filepath);

			menu->marked = true;
			menu->selected = false;
		}
		break;
	case MENU_EVENT_DESELECTED:
		break;
	case MENU_EVENT_BEFORE_DELETE:
		break;
	case MENU_EVENT_NONE:
	default:
		break;
	}
}

static void record_menu_load_node_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED:
		break;
	case MENU_EVENT_DEACTIVATED:
		break;
	case MENU_EVENT_SELECTED:
		if (menu->marked) {
			lg_recorder_mode = RECORDER_MODE_IDLE;

			char cmd[512];
			sprintf(cmd, PLUGIN_NAME ".stop");
			lg_plugin_host->send_command(cmd);
			printf("stop loading\n");

			menu->marked = false;
			menu->selected = false;
		} else if (lg_recorder_mode == RECORDER_MODE_IDLE) {
			lg_recorder_mode = RECORDER_MODE_PLAY;

			char filepath[256];
			sprintf(filepath, "%s/%s", lg_record_path, (char*) menu->user_data);
			char cmd[512];
			sprintf(cmd, PLUGIN_NAME ".play %s", filepath);
			lg_plugin_host->send_command(cmd);
			printf("start loading %s\n", filepath);

			menu->marked = true;
			menu->selected = false;
		}
		break;
	case MENU_EVENT_DESELECTED:
		break;
	case MENU_EVENT_BEFORE_DELETE:
		if (menu->user_data) {
			free(menu->user_data);
		}
		break;
	case MENU_EVENT_NONE:
	default:
		break;
	}
}

static void record_menu_load_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED: {
		struct dirent *d;
		DIR *dir;

		dir = opendir(lg_record_path);
		if (dir != NULL) {
			while ((d = readdir(dir)) != 0) {
				if (d->d_name[0] != L'.' && d->d_type == DT_DIR) {
					char *name = malloc(256);
					strncpy(name, d->d_name, 256);
					MENU_T *node_menu = menu_new(name,
							record_menu_load_node_callback, name);
					menu_add_submenu(menu, node_menu, INT_MAX);
				}
			}
		}
		break;
	}
	case MENU_EVENT_DEACTIVATED: {
		for (int idx = 0; menu->submenu[idx]; idx++) {
			menu_delete(&menu->submenu[idx]);
		}
		break;
	}
	case MENU_EVENT_SELECTED:
		break;
	case MENU_EVENT_DESELECTED:
		break;
	case MENU_EVENT_BEFORE_DELETE:
		break;
	case MENU_EVENT_NONE:
	default:
		break;
	}
}

static void init_menu() {
	MENU_T *menu = lg_plugin_host->get_menu();
	{
		MENU_T *sub_menu = menu_add_submenu(menu,
				menu_new("ImageRecorder", NULL, NULL), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new("Record", record_menu_record_callback, NULL), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new("Play", record_menu_load_callback, NULL), INT_MAX);
	}
}

static void init_options(void *user_data, json_t *_options) {
	PLUGIN_T *plugin = (PLUGIN_T*) user_data;
	json_t *options = json_object_get(_options, PLUGIN_NAME);
	if (options == NULL) {
		return;
	}
	{
		json_t *obj = json_object_get(options, "record_path");
		if (obj) {
			strncpy(lg_record_path, json_string_value(obj),
					sizeof(lg_record_path) - 1);

			init_menu();
		}
	}
}

static void save_options(void *user_data, json_t *_options) {
	json_t *options = json_object();
	json_object_set_new(_options, PLUGIN_NAME, options);

	if (lg_record_path[0] != '\0') {
		json_object_set_new(options, "record_path",
				json_string(lg_record_path));
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
		strcpy(factory->name, STREAMER_NAME);
		factory->release = release;
		factory->create_vstreamer = create_vstreamer;
		factory->user_data = factory;

		lg_plugin_host->add_vstreamer_factory(factory);
	}
}
