#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include "plugin_template.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PLUGIN_NAME "plugin_template"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

//params
static float lg_param1 = 0.0f;
static float lg_param2 = 0.0f;

static bool lg_is_init = false;

static void init() {
	if (lg_is_init) {
		return;
	} else {
		lg_is_init = true;
	}
}

static void release(void *user_data) {
	free(user_data);
}

static int command_handler(void *user_data, const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, PLUGIN_NAME ".set_param1", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			int len = sscanf(param, "%f", &value);
			if (len == 1) {
				lg_param1 = value;
			}
			printf("set_param1 : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_param2", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			int len = sscanf(param, "%f", &value);
			if (len == 1) {
				lg_param2 = value;
			}
			printf("set_param2 : completed\n");
		}
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
	switch (node_id) {
	case PICAM360_HOST_NODE_ID:
		break;
	default:
		break;
	}
}

static void init_options(void *user_data, json_t *options) {
	lg_param1 = json_number_value(
			json_object_get(options, PLUGIN_NAME ".param1"));
	lg_param2 = json_number_value(
			json_object_get(options, PLUGIN_NAME ".param2"));
}

static void save_options(void *user_data, json_t *options) {
	json_object_set_new(options, PLUGIN_NAME ".param1", json_real(lg_param1));
	json_object_set_new(options, PLUGIN_NAME ".param2", json_real(lg_param2));
}

#define MAX_INFO_LEN 1024
static char lg_info[MAX_INFO_LEN];
static char *get_info(void *user_data) {
	int cur = 0;
	cur += snprintf(lg_info, MAX_INFO_LEN, L"param1=%f,param2=%f", lg_param1,
			lg_param2);
	return lg_info;
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	init();
	lg_plugin_host = plugin_host;

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(PLUGIN_T));
		strcpy(plugin->name, PLUGIN_NAME);
		plugin->release = release;
		plugin->command_handler = command_handler;
		plugin->event_handler = event_handler;
		plugin->init_options = init_options;
		plugin->save_options = save_options;
		plugin->get_info = get_info;
		plugin->user_data = plugin;

		*_plugin = plugin;
	}
}
