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
#define STREAMER_NAME "view_point_multiplexor"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static PLUGIN_HOST_T *lg_plugin_host = NULL;

static void release(void *obj) {

	free(obj);
}

static int _command_handler(int argc, char *argv[]) {
	int opt;
	int ret = 0;
	char *cmd = argv[0];
	if (cmd == NULL) {
		//do nothing
	} else if (strcmp(cmd, "generate") == 0) {
		int keyframe_interval = 1;
		int *keyframe_offset_ary;
		int fps = 10;
		int fov = 120;
		char tmp_path[257] = { };
		char *i_str = NULL; //input
		char *r_str = NULL; //rendering
		char *e_str = NULL; //encode
		char *o_str = NULL; //output file
		int n = 3;
		optind = 1; // reset getopt
		while ((opt = getopt(argc, argv, "i:r:e:o:n:f:k:")) != -1) {
			switch (opt) {
			case 'i':
				i_str = optarg;
				break;
			case 'r':
				r_str = optarg;
				break;
			case 'e':
				e_str = optarg;
				break;
			case 'o':
				o_str = optarg;
				snprintf(tmp_path, sizeof(tmp_path) - 1, "%s.tmp", o_str);
				break;
			case 'n':
				sscanf(optarg, "%d", &n);
				break;
			case 'f':
				sscanf(optarg, "%d", &fps);
				break;
			case 'k':
				sscanf(optarg, "%d", &keyframe_interval);
				break;
			}
		}
		if (i_str == NULL || r_str == NULL || e_str == NULL || e_str == NULL
				|| o_str == NULL) {
			return -1;
		}
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
			keyframe_offset_ary = (int*) malloc(sizeof(int) * i);
		}
		{ //config
			char path[512];
			sprintf(path, "%s/config.json", tmp_path);
			ret = mkdir_path(path, 0775);

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
						json_object_set_new(obj, view_angle,
								json_integer(keyframe_offset_ary[i]));
					}
				}
				json_object_set_new(options, "keyframe_offset", obj);
			}
			json_dump_file(options, path,
					JSON_PRESERVE_ORDER | JSON_INDENT(4)
							| JSON_REAL_PRECISION(9));
			json_decref(options);
		}
		{
			int i = 0;
			int roll = 0;
			int split_p = n * 2;
			for (int p = 0; p <= split_p; p++) {
				int _p = (p <= n) ? p : n * 2 - p;
				int split_y = (_p == 0) ? 1 : 4 * _p;
				for (int y = 0; y < split_y; y++, i++) {
					int pitch = 180 * p / split_p;
					int yaw = 360 * y / split_y;
					char view_angle[32];
					sprintf(view_angle, "%d_%d", pitch, yaw);

					VECTOR4D_T vq = quaternion_init();
					vq = quaternion_multiply(vq,
							quaternion_get_from_y(yaw * M_PI / 180));
					vq = quaternion_multiply(vq,
							quaternion_get_from_x(pitch * M_PI / 180));
					vq = quaternion_multiply(vq,
							quaternion_get_from_y(45 * M_PI / 180)); //horizon_opt

					int buff_size = 1024;
					buff_size += strlen(i_str);
					buff_size += strlen(r_str);
					buff_size += strlen(e_str);
					buff_size += strlen(tmp_path);
					char *def = (char*) malloc(buff_size);

					int len = 0;
					len += snprintf(def + len, buff_size - len, "%s", i_str);
					len += snprintf(def + len, buff_size - len,
							"!%s view_quat=%.3f,%.3f,%.3f,%.3f fov=%d", r_str,
							vq.x, vq.y, vq.z, vq.w, fov);
					len += snprintf(def + len, buff_size - len, "!%s", e_str);
					len += snprintf(def + len, buff_size - len,
							"!image_recorder base_path=%s/%s mode=RECORD",
							tmp_path, view_angle); //x_y
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
							break;
						}
						usleep(100 * 1000);
					}
					lg_plugin_host->destroy_vstream(uuid);
				}
			}
		}
		{ //pvf archive
			int ret;
			char buff[256];
			snprintf(buff, sizeof(buff), "(cd %s && zip -0r - *) > %s",
					tmp_path, o_str);
			ret = system(buff);
			snprintf(buff, sizeof(buff), "rm -rf %s", tmp_path);
			ret = system(buff);
		}
		{ //finalize
			free(keyframe_offset_ary);
		}
		printf("%s : completed\n", cmd);
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
}

static void save_options(void *user_data, json_t *_options) {
	json_t *options = json_object();
	json_object_set_new(_options, PLUGIN_NAME, options);

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
}
