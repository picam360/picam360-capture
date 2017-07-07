#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <dirent.h>
#include <stdbool.h>
#include <pthread.h>

#include "kokuyoseki.h"

static KOKUYOSEKI_CALLBACK lg_kokuyoseki_callback = NULL;

static PLUGIN_HOST_T *lg_plugin_host = NULL;

void set_kokuyoseki_callback(KOKUYOSEKI_CALLBACK callback) {
	lg_kokuyoseki_callback = callback;
}

static int read_hex(const char * const filename) {
	FILE *in;
	unsigned int value;

	in = fopen(filename, "rb");
	if (!in)
		return -1;

	if (fscanf(in, "%x", &value) == 1) {
		fclose(in);
		return (int) value;
	}

	fclose(in);
	return -1;
}

static bool lg_stop_thread = false;
static void *poling_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "KOKUYOSEKI");

	struct dirent *d;
	DIR *dir;
	char fileName[256];
	char *kokuyoseki_event = NULL;

	// Open /dev directory
	dir = opendir("/sys/class/input");

	// Iterate over /dev files
	while ((d = readdir(dir)) != 0) {
		int vendor = 0;
		int product = 0;
		{
			sprintf(fileName, "/sys/class/input/%s/device/id/vendor",
					d->d_name);
			vendor = read_hex(fileName);
		}
		{
			sprintf(fileName, "/sys/class/input/%s/device/id/product",
					d->d_name);
			product = read_hex(fileName);
		}

		if (vendor == KOKUYOSEKI_VENDOR && product == KOKUYOSEKI_PRODUCT) {
			sprintf(fileName, "/dev/input/%s", d->d_name);
			kokuyoseki_event = fileName;
			break;
		}
	}

	closedir(dir);

	if (kokuyoseki_event == NULL) {
		return NULL;
	}
	int fd = open(kokuyoseki_event, O_RDWR);
	if (fd < 0) {
		return NULL;
	}
	ioctl(fd, EVIOCGRAB, 1);
	while (!lg_stop_thread) {
		struct input_event event;

		if (read(fd, &event, sizeof(event)) != sizeof(event)) {
			perror("error : on read\n");
			return NULL;
		}
		if (event.type == 1) {
			if (lg_kokuyoseki_callback) {
				lg_kokuyoseki_callback(event.time, event.code, event.value);
			}
		}
	}
	return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Scan /dev looking for hidraw devices and then check to see if each is a kokuyoseki
/////////////////////////////////////////////////////////////////////////////////////////////
void open_kokuyoseki() {
	lg_stop_thread = false;
	pthread_t poling_thread;
	pthread_create(&poling_thread, NULL, poling_thread_func, (void*) NULL);
	return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void close_kokuyoseki() {
	lg_stop_thread = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//plugin
/////////////////////////////////////////////////////////////////////////////////////////////
static struct timeval lg_last_kokuyoseki_time = { };
static int lg_last_button = -1;

static void release(void *user_data) {
	free(user_data);
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

static void kokuyoseki_callback(struct timeval time, int button, int value) {
	if (value == 1) {
		return;
	}
	struct timeval diff;
	timersub(&time, &lg_last_kokuyoseki_time, &diff);
	float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
	MENU_T *menu = lg_plugin_host->get_menu();
	if (!menu->activated) {
		switch (button) {
		case NEXT_BUTTON:
			lg_plugin_host->send_event(PICAM360_CONTROLLER_NODE_ID, CONTROLLER_EVENT_NEXT);
			break;
		case BACK_BUTTON:
			lg_plugin_host->send_event(PICAM360_CONTROLLER_NODE_ID, CONTROLLER_EVENT_BACK);
			break;
		case BLACKOUT_BUTTON:
			if (!lg_plugin_host->get_menu_visible()) {
				lg_plugin_host->set_menu_visible(true);
			} else {
				menu_operate(menu, MENU_OPERATE_ACTIVE_NEXT); //open menu
			}
			break;
		case BACK_BUTTON_LONG:
			lg_plugin_host->set_menu_visible(false);
			break;
		}
	} else {
		switch (button) {
		case BACK_BUTTON_LONG:
			lg_plugin_host->set_menu_visible(false);
			break;
		case NEXT_BUTTON:
			menu_operate(menu, MENU_OPERATE_SELECT);
			break;
		case BACK_BUTTON:
			menu_operate(menu, MENU_OPERATE_DESELECT);
			break;
		case BLACKOUT_BUTTON:
			menu_operate(menu, MENU_OPERATE_ACTIVE_NEXT);
			break;
		default:
			break;
		}
	}
	lg_last_kokuyoseki_time = time;
	lg_last_button = button;
}

static wchar_t *get_info(void *user_data) {
	return NULL;
}

static bool is_init = false;
static void init() {
	if (is_init) {
		return;
	}
	is_init = true;

	set_kokuyoseki_callback((KOKUYOSEKI_CALLBACK) kokuyoseki_callback);
	open_kokuyoseki();
}

void create_kokuyoseki(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
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
