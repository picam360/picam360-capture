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

#include "video_reciever.h"

#define PLUGIN_NAME "video_reciever"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

typedef struct _video_reciever {
	DECODER_T super;

	void *user_data;
} video_reciever;

static void release(void *obj) {
	free(obj);
}
static int rtp_callback(unsigned char *data, unsigned int data_len, unsigned char pt, unsigned int seq_num) {
	return 0;
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
		DECODER_FACTORY_T *decoder_factory = (DECODER_FACTORY_T*) malloc(sizeof(DECODER_FACTORY_T));
		memset(decoder_factory, 0, sizeof(DECODER_FACTORY_T));
		strcpy(decoder_factory->name, DECODER_NAME);
		decoder_factory->release = release;

		lg_plugin_host->add_decoder_factory(decoder_factory);
	}
}
