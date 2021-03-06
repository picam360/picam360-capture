#define _GNU_SOURCE
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
#include <wordexp.h>

#include "view_point_multiplexor.h"
#include "mrevent.h"
#include "tools.h"

#define PLUGIN_NAME "view_point_multiplexor"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define STATUS_VAR(name) lg_status_ ## name
#define STATUS_INIT(plugin_host, prefix, name) STATUS_VAR(name) = new_status(prefix #name); \
                                               (plugin_host)->add_status(STATUS_VAR(name));
//status to downstream
static STATUS_T *STATUS_VAR(status);

typedef struct _vpm_plugin_t {
	PLUGIN_T super;

	char status_str[256];

	bool run;
	pthread_t streaming_thread;

	//convert
	struct {
		int frame_pack_size;
		int keyframe_interval;
		int fov;
		int fps;
		char i_str[256]; //input
		char r_str[256]; //rendering
		char e_str[256]; //encode
		char o_str[256]; //output file
		int n;
		bool horizon_opt;
		bool resume;
	} params;
} vpm_plugin_t;

vpm_plugin_t *lg_plugin;

static void status_get_value(void *user_data, char *buff, int buff_len) {
	vpm_plugin_t *_this = lg_plugin;

	STATUS_T *status = (STATUS_T*) user_data;
	if (status == STATUS_VAR(status)) {
		snprintf(buff, buff_len, "%s", _this->status_str);
	}
}
static void status_set_value(void *user_data, const char *value) {
}
static void status_release(void *user_data) {
	free(user_data);
}
static STATUS_T* new_status(const char *name) {
	STATUS_T *status = (STATUS_T*) malloc(sizeof(STATUS_T));
	strcpy(status->name, name);
	status->get_value = status_get_value;
	status->set_value = status_set_value;
	status->release = status_release;
	status->user_data = status;
	return status;
}

static PLUGIN_HOST_T *lg_plugin_host = NULL;
static char lg_frame_packer_path[512];

static void release(void *obj) {

	free(obj);
}

static int pvf_archive(const char *tmp_path, const char *o_str) {
	int ret;
	char buff[256];
	snprintf(buff, sizeof(buff), "(cd %s && zip -0r - *) > %s_", tmp_path,
			o_str);
	ret = system(buff);
	snprintf(buff, sizeof(buff), "mv %s_ %s", o_str, o_str);
	ret = system(buff);
	snprintf(buff, sizeof(buff), "rm -rf %s", tmp_path);
	ret = system(buff);
	return 0;
}
static void* streaming_thread_fnc(void *obj) {
	vpm_plugin_t *_this = lg_plugin;

	int ret = 0;

	int frame_pack_size = _this->params.frame_pack_size;
	int keyframe_interval = _this->params.keyframe_interval;
	int fov = _this->params.fov;
	int fps = _this->params.fps;
	char *i_str = _this->params.i_str; //input
	char *r_str = _this->params.r_str; //rendering
	char *e_str = _this->params.e_str; //encode
	char *o_str = _this->params.o_str; //output file
	int n = _this->params.n;
	bool horizon_opt = _this->params.horizon_opt;
	bool resume = _this->params.resume;

	int num_of_viewangle = 0;
	int *keyframe_offset_ary;
	int i_start = 0;
	char tmp_path[257];
	snprintf(tmp_path, sizeof(tmp_path) - 1, "%s.tmp", o_str);

	{ //get num of view angles
		int i = 0;
		int split_p = n * 2;
		for (int p = 0; p <= split_p; p++) {
			int _p = (p <= n) ? p : n * 2 - p;
			int split_y = (_p == 0) ? 1 : 4 * _p;
			for (int y = 0; y < split_y; y++) {
				i++;
			}
		}
		num_of_viewangle = i;
		keyframe_offset_ary = (int*) malloc(sizeof(int) * num_of_viewangle);
	}
	{ //config
		char path[512];
		sprintf(path, "%s/config.json", tmp_path);
		json_error_t error;
		json_t *options = NULL;
		if (resume) {
			options = json_load_file(path, 0, &error);
		}
		if (options) {
			int _frame_pack_size = json_number_value(
					json_object_get(options, "frame_pack_size"));
			if (_frame_pack_size > 0) { // bson pack has already done
				pvf_archive(tmp_path, o_str);
				if (keyframe_offset_ary) { //finalize
					free(keyframe_offset_ary);
				}
				printf("generate : completed\n");
				return 0;
			}

			n = json_number_value(json_object_get(options, "num_per_quarter"));
			fps = json_number_value(json_object_get(options, "fps"));
			keyframe_interval = json_number_value(
					json_object_get(options, "keyframe_interval"));
			{
				json_t *obj = json_object_get(options, "keyframe_offset");
				if (obj == NULL || json_is_object(obj) == 0) {
					printf("can not resume\n");
					exit(-1);
				}
				int i = 0;
				int split_p = n * 2;
				bool start_position_found = false;
				for (int p = 0; p <= split_p; p++) {
					int _p = (p <= n) ? p : n * 2 - p;
					int split_y = (_p == 0) ? 1 : 4 * _p;
					for (int y = 0; y < split_y; y++, i++) {
						int pitch = 180 * p / split_p;
						int yaw = 360 * y / split_y;
						char view_angle[32];
						sprintf(view_angle, "%d_%d", pitch, yaw);
						json_t *obj2 = json_object_get(obj, view_angle);
						if (obj2 == NULL) {
							printf("can not resume\n");
							exit(-1);
						}
						keyframe_offset_ary[i] = json_number_value(obj2);
						if (!start_position_found) {
							char dirname[512];
							sprintf(dirname, "%s/%s/", tmp_path, view_angle);
							struct stat buffer;
							ret = stat(dirname, &buffer);
							if (ret != 0) {
								i_start = MAX(i - 1, 0);
								start_position_found = true;
							}
						}
					}
				}
			}
		} else {
			ret = mkdir_path(path, 0777);

			json_t *options = json_object();
			json_object_set_new(options, "num_per_quarter", json_integer(n));
			json_object_set_new(options, "fps", json_integer(fps));
			json_object_set_new(options, "keyframe_interval",
					json_integer(keyframe_interval));
			{
				json_t *obj = json_object();
				int i = 0;
				int split_p = n * 2;
				for (int p = 0; p <= split_p; p++) {
					int _p = (p <= n) ? p : n * 2 - p;
					int split_y = (_p == 0) ? 1 : 4 * _p;
					for (int y = 0; y < split_y; y++, i++) {
						int pitch = 180 * p / split_p;
						int yaw = 360 * y / split_y;
						char view_angle[32];
						sprintf(view_angle, "%d_%d", pitch, yaw);
						keyframe_offset_ary[i] = (int) (rand()
								% keyframe_interval);
						if (frame_pack_size > 0) { // sync_frame_pack_size
							keyframe_offset_ary[i] /= frame_pack_size;
							keyframe_offset_ary[i] *= frame_pack_size;
						}
						json_object_set_new(obj, view_angle,
								json_integer(keyframe_offset_ary[i]));
					}
				}
				json_object_set_new(options, "keyframe_offset", obj);
			}
			json_dump_file(options, path,
					JSON_PRESERVE_ORDER | JSON_INDENT(4)
							| JSON_REAL_PRECISION(9));
		}
		json_decref(options);
	}
	{ // progress
		int progress = 100 * i_start / num_of_viewangle;
		sprintf(_this->status_str, "CONVERT=%d", progress);
	}
	{
		int i = 0;
		int split_p = n * 2;
		for (int p = 0; p <= split_p; p++) {
			int _p = (p <= n) ? p : n * 2 - p;
			int split_y = (_p == 0) ? 1 : 4 * _p;
			for (int y = 0; y < split_y; y++, i++) {
				if (i < i_start) {
					continue;
				}
				int pitch = 180 * p / split_p;
				int yaw = 360 * y / split_y;
				char view_angle[32];
				sprintf(view_angle, "%d_%d", pitch, yaw);

				{ // mkdir
					char dirname[512];
					sprintf(dirname, "%s/%s/", tmp_path, view_angle);
					struct stat buffer;
					ret = stat(dirname, &buffer);
					if (ret != 0) {
						ret = mkdir_path(dirname, 0777);
					}
				}

				if (!_this->run) { //stop
					if (keyframe_offset_ary) { //finalize
						free(keyframe_offset_ary);
					}
					printf("generate : stopped\n");

					return NULL;
				}

				VECTOR4D_T vq = quaternion_init();
				vq = quaternion_multiply(vq,
						quaternion_get_from_y(yaw * M_PI / 180));
				vq = quaternion_multiply(vq,
						quaternion_get_from_x(pitch * M_PI / 180));
				if (horizon_opt) {
					vq = quaternion_multiply(vq,
							quaternion_get_from_y(45 * M_PI / 180)); //horizon_opt
				}

				int buff_size = 1024;
				buff_size += strlen(i_str);
				buff_size += strlen(r_str);
				buff_size += strlen(e_str);
				buff_size += strlen(tmp_path);
				char *def = (char*) malloc(buff_size);

				int len = 0;
				len += snprintf(def + len, buff_size - len, "%s", i_str);
				len += snprintf(def + len, buff_size - len,
						"!%s view_quat=%.3f,%.3f,%.3f,%.3f fov=%d", r_str, vq.x,
						vq.y, vq.z, vq.w, fov);
				len += snprintf(def + len, buff_size - len, "!%s", e_str);
				len += snprintf(def + len, buff_size - len,
						"!image_recorder base_path=%s/%s mode=RECORD", tmp_path,
						view_angle); //x_y
				{
					char str[32];
					sprintf(str, "%d", keyframe_interval);
					strchg(def, "@keyframe_interval@", str);
				}
				{
					char str[32];
					sprintf(str, "%d", keyframe_offset_ary[i]);
					strchg(def, "@keyframe_offset@", str);
				}

				uuid_t uuid;
				uuid_generate(uuid);

				VSTREAMER_T *vstreamer = lg_plugin_host->build_vstream(uuid,
						def);
				vstreamer->start(vstreamer);
				printf("started : %s\n", def);
				free(def);

				while (1) {
					char eof_str[8];
					vstreamer->get_param(vstreamer, "eof", eof_str,
							sizeof(eof_str));
					if (eof_str[0] == '1' || eof_str[0] == 't'
							|| eof_str[0] == 'T') {
						bool is_end = true;
						for (VSTREAMER_T *stream = vstreamer->next_streamer;
								stream != NULL; stream =
										stream->next_streamer) {
							char frames_in_buffer_str[8] = { };
							stream->get_param(stream, "frames_in_buffer",
									frames_in_buffer_str,
									sizeof(frames_in_buffer_str));
							if (frames_in_buffer_str[0] == 0
									|| frames_in_buffer_str[0] == '0') {
								is_end = true;
							} else {
								is_end = false;
								break;
							}
						}
						if (is_end) {
							break;
						}
					}
					usleep(100 * 1000);
				}
				lg_plugin_host->destroy_vstream(uuid);

				{ // progress
					int progress = 100 * (i + 1) / num_of_viewangle;
					sprintf(_this->status_str, "CONVERT=%d", progress);
				}
			}
		}
	}
	if (frame_pack_size > 0) { // frame pack
		int ret;
		char buff[256];
		snprintf(buff, sizeof(buff), "node %s %d %s %s_", lg_frame_packer_path,
				frame_pack_size, tmp_path, tmp_path);
		ret = system(buff);
		snprintf(buff, sizeof(buff), "rm -rf %s", tmp_path);
		ret = system(buff);
		snprintf(buff, sizeof(buff), "mv %s_ %s", tmp_path, tmp_path);
		ret = system(buff);
	}
	pvf_archive(tmp_path, o_str);
	if (keyframe_offset_ary) { //finalize
		free(keyframe_offset_ary);
	}
	printf("generate : completed\n");
	{ // progress
		strcpy(_this->status_str, "DONE");
	}

	_this->run = false;
	return NULL;
}

static int _command_handler(int argc, char *argv[]) {
	vpm_plugin_t *_this = lg_plugin;

	int opt;
	int ret = 0;
	char *cmd = argv[0];
	if (cmd == NULL) {
		//do nothing
	} else if (strcmp(cmd, "reset") == 0) {
		if (strcmp(_this->status_str, "DONE") == 0) {
			strcpy(_this->status_str, "IDLE");
		}
	} else if (strcmp(cmd, "stop") == 0) {
		if (_this->run == true) {
			_this->run = false;
			pthread_join(_this->streaming_thread, NULL);
			strcpy(_this->status_str, "IDLE");
		}
	} else if (strcmp(cmd, "generate") == 0) {
		if (_this->run == true) {
			ret = pthread_tryjoin_np(_this->streaming_thread, NULL);
			if (ret != 0) {
				printf("now generating another...\n");
				return 0;
			}
			_this->run = false; // fail safe : this should be done in working thread
		}
		memset(&_this->params, 0, sizeof(_this->params));
		_this->params.frame_pack_size = 0;
		_this->params.keyframe_interval = 1;
		_this->params.fov = 120;
		_this->params.fps = 10;
		_this->params.n = 3;
		_this->params.horizon_opt = true;
		_this->params.resume = false;

		optind = 1; // reset getopt
		while ((opt = getopt(argc, argv, "i:r:e:o:n:v:f:k:p:h:c")) != -1) {
			switch (opt) {
			case 'i':
				strncpy(_this->params.i_str, optarg,
						sizeof(_this->params.i_str));
				break;
			case 'r':
				strncpy(_this->params.r_str, optarg,
						sizeof(_this->params.r_str));
				break;
			case 'e':
				strncpy(_this->params.e_str, optarg,
						sizeof(_this->params.e_str));
				break;
			case 'o':
				strncpy(_this->params.o_str, optarg,
						sizeof(_this->params.o_str));
				break;
			case 'n':
				sscanf(optarg, "%d", &_this->params.n);
				break;
			case 'v':
				sscanf(optarg, "%d", &_this->params.fov);
				break;
			case 'f':
				sscanf(optarg, "%d", &_this->params.fps);
				break;
			case 'k':
				sscanf(optarg, "%d", &_this->params.keyframe_interval);
				break;
			case 'p':
				sscanf(optarg, "%d", &_this->params.frame_pack_size);
				break;
			case 'h':
				if (optarg[0] == '0' || optarg[0] == 'f') {
					_this->params.horizon_opt = false;
				}
				break;
			case 'c':
				_this->params.resume = true;
				break;
			}
		}
		if (_this->params.i_str == NULL || _this->params.r_str == NULL
				|| _this->params.e_str == NULL || _this->params.o_str == NULL) {
			return -1;
		}
		_this->run = true;
		pthread_create(&_this->streaming_thread, NULL, streaming_thread_fnc,
				NULL);
	}
}

static int command_handler(void *user_data, const char *_buff) {
	int opt;
	int ret = 0;
	char buff[512];
	strncpy(buff, _buff, sizeof(buff));

	wordexp_t p;
	ret = wordexp(buff, &p, 0);
	if (ret != 0) {
		printf("command parse error : %s", buff);
	} else {
		ret = _command_handler(p.we_wordc, p.we_wordv);
	}
	wordfree(&p);

	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
}

static void init_options(void *user_data, json_t *_options) {
	PLUGIN_T *plugin = (PLUGIN_T*) user_data;
	json_t *options = json_object_get(_options, PLUGIN_NAME);
	if (options == NULL) {
		return;
	}
	json_t *value = json_object_get(options, "frame_packer_path");
	if (value) {
		int len = json_string_length(value);
		if (len < sizeof(lg_frame_packer_path)) {
			strncpy(lg_frame_packer_path, json_string_value(value), len);
		}
	}
}

static void save_options(void *user_data, json_t *_options) {
	json_t *options = json_object();
	json_object_set_new(_options, PLUGIN_NAME, options);

	json_object_set_new(options, "frame_packer_path",
			json_string(lg_frame_packer_path));
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	lg_plugin_host = plugin_host;

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(vpm_plugin_t));
		memset(plugin, 0, sizeof(vpm_plugin_t));
		strcpy(plugin->name, PLUGIN_NAME);
		plugin->release = release;
		plugin->command_handler = command_handler;
		plugin->event_handler = event_handler;
		plugin->init_options = init_options;
		plugin->save_options = save_options;
		plugin->get_info = NULL;
		plugin->user_data = plugin;

		*_plugin = plugin;
		lg_plugin = (vpm_plugin_t*) plugin;
	}
	strcpy(lg_plugin->status_str, "IDLE");
	STATUS_INIT(plugin_host, PLUGIN_NAME ".", status);
}
