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
#include <math.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#if __linux
#include <sys/prctl.h>
#endif

#include "create_plugin.h"
#include "mjpeg_omx_decoder.h"
#include "window_gl_renderer.h"
#include "picam360map_gl_renderer.h"
#include "equirectangular_gl_renderer.h"
#include "omx_encoder.h"

#define PLUGIN_NAME "picam360gl"

#define MAX_SUB_PLUGINS 16
typedef struct _picam360gl_plugin {
	PLUGIN_T super;

	PLUGIN_T *sub_plugins[MAX_SUB_PLUGINS] = { };

	void *user_data;
} picam360gl_plugin;

static char lg_license_path[256] = { };

static int command_handler(void *user_data, const char *buff) {
	picam360gl_plugin *plugin = (picam360gl_plugin*) user_data;
	for (int i = 0; i < MAX_SUB_PLUGINS && plugin->sub_plugins[i] != NULL; i++) {
		int name_len = strlen(plugin->sub_plugins[i]->name);
		if (strncmp(buff, plugin->sub_plugins[i]->name, name_len) == 0 && buff[name_len] == '.') {
			plugin->sub_plugins[i]->command_handler(plugin->sub_plugins[i], buff + name_len + 1);
		}
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
	picam360gl_plugin *plugin = (picam360gl_plugin*) user_data;
	for (int i = 0; i < MAX_SUB_PLUGINS && plugin->sub_plugins[i] != NULL; i++) {
		if (plugin->sub_plugins[i]->event_handler) {
			plugin->sub_plugins[i]->event_handler(plugin->sub_plugins[i], node_id, event_id);
		}
	}
}

static void init_options(void *user_data, json_t *_options) {
	picam360gl_plugin *plugin = (picam360gl_plugin*) user_data;
	json_t *options = json_object_get(_options, PLUGIN_NAME);
	for (int i = 0; i < MAX_SUB_PLUGINS && plugin->sub_plugins[i] != NULL; i++) {
		if (plugin->sub_plugins[i]->init_options) {
			plugin->sub_plugins[i]->init_options(plugin->sub_plugins[i], options);
		}
	}
}

static void save_options(void *user_data, json_t *_options) {
	picam360gl_plugin *plugin = (picam360gl_plugin*) user_data;
	json_t *options = json_object();
	for (int i = 0; i < MAX_SUB_PLUGINS && plugin->sub_plugins[i] != NULL; i++) {
		if (plugin->sub_plugins[i]->save_options) {
			plugin->sub_plugins[i]->save_options(plugin->sub_plugins[i], options);
		}
	}
	json_object_set_new(_options, PLUGIN_NAME, options);
}

static void release(void *obj) {
	free(obj);
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(picam360gl_plugin));
		memset(plugin, 0, sizeof(picam360gl_plugin));
		strcpy(plugin->name, PLUGIN_NAME);
		plugin->release = release;
		plugin->command_handler = command_handler;
		plugin->event_handler = event_handler;
		plugin->init_options = init_options;
		plugin->save_options = save_options;
		plugin->get_info = NULL;
		plugin->user_data = plugin;

		PLUGIN_T **sub_plugins = ((picam360gl_plugin*) plugin)->sub_plugins;
		int idx = 0;
		{
			create_mjpeg_decoder_plugin(plugin_host, &sub_plugins[idx]);
			sub_plugins[idx]->parent = plugin;
			idx++;
		}
		{
			create_window_plugin(plugin_host, &sub_plugins[idx]);
			sub_plugins[idx]->parent = plugin;
			idx++;
		}
//		{
//			create_picam360map_plugin(plugin_host, &sub_plugins[idx]);
//			sub_plugins[idx]->parent = plugin;
//			idx++;
//		}
//		{
//			create_equirectangular_plugin(plugin_host, &sub_plugins[idx]);
//			sub_plugins[idx]->parent = plugin;
//			idx++;
//		}
//		{
//			create_omx_encoder_plugin(plugin_host, &sub_plugins[idx]);
//			sub_plugins[idx]->parent = plugin;
//			idx++;
//		}

		*_plugin = plugin;
	}
}
