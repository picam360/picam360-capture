#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdbool.h>
#include <fcntl.h>
#include <pthread.h>
#include <editline/readline.h>
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <uuid/uuid.h>
#include <limits.h>

#include "menu_handler.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PACKET_FOLDER_PATH "/media/usbdisk/packet"
#define STILL_FOLDER_PATH "/media/usbdisk/still"
#define VIDEO_FOLDER_PATH "/media/usbdisk/video"

enum SYSTEM_CMD {
	SYSTEM_CMD_NONE, SYSTEM_CMD_SHUTDOWN, SYSTEM_CMD_REBOOT, SYSTEM_CMD_EXIT,
};

enum CALIBRATION_CMD {
	CALIBRATION_CMD_NONE,
	CALIBRATION_CMD_SAVE,
	CALIBRATION_CMD_IMAGE_CIRCLE,
	CALIBRATION_CMD_IMAGE_PARAMS,
	CALIBRATION_CMD_VIEWER_COMPASS,
	CALIBRATION_CMD_VEHICLE_COMPASS,
};

static PICAM360CAPTURE_T *state = NULL;

static int lg_resolution = 4;
static bool lg_stereo_enabled = false;
static bool lg_sync_enabled = true;

static char lg_convert_base_path[256];
static uint32_t lg_convert_frame_num = 0;
static bool lg_is_converting = false;

static void _get_menu_str(char **buff, int buff_len, MENU_T *menu, int depth) {
	int len = snprintf(*buff, buff_len, "%s,%d,%d,%d,%d\n", menu->name, depth,
			menu->activated ? 1 : 0, menu->selected ? 1 : 0,
			menu->marked ? 1 : 0);
	*buff += len;
	buff_len -= len;
	depth++;
	if (menu->selected) {
		for (int idx = 0; menu->submenu[idx]; idx++) {
			_get_menu_str(buff, buff_len, menu->submenu[idx], depth);
		}
	}
}

void get_menu_str(char *_buff, int buff_len) {
	char **buff = &_buff;
	int len = snprintf(*buff, buff_len,
			"name,depth,activated,selected,marked\n");
	*buff += len;
	buff_len -= len;
	_get_menu_str(buff, buff_len, state->menu, 0);
}

static int get_last_id(const char *path) {
	struct dirent *d;
	DIR *dir;

	dir = opendir(path);
	if (dir != NULL) {
		int last_id = 0;
		while ((d = readdir(dir)) != 0) {
			if (d->d_name[0] != L'.') {
				int id = 0;
				sscanf(d->d_name, "%d", &id);
				if (id > last_id) {
					last_id = id;
				}
			}
		}
		closedir(dir);
		return last_id;
	} else {
		return -1;
	}
}

static void loading_callback(void *user_data, int ret) {
	lg_is_converting = false;
	printf("end of loading\n");
}

static void convert_snap_handler() {
	if (lg_is_converting) {
		rtp_increment_loading(state->rtp, 100 * 1000); //10 fps

		char dst[256];
		snprintf(dst, 256, "%s/%d.jpeg", lg_convert_base_path,
				lg_convert_frame_num);
		lg_convert_frame_num++;
		state->plugin_host.snap(lg_resolution * 1024, lg_resolution * 512,
				RENDERING_MODE_EQUIRECTANGULAR, dst);
	}
}

static void packet_menu_record_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED:
		break;
	case MENU_EVENT_DEACTIVATED:
		break;
	case MENU_EVENT_SELECTED:
		menu->selected = false;
		if (lg_is_converting) { //stop convert
			lg_is_converting = false;

			rtp_set_auto_play(state->rtp, true);
			rtp_set_is_looping(state->rtp, true);

			snprintf(menu->name, 8, "Record");
			printf("stop converting\n");
			menu->selected = false;
		} else if (rtp_is_loading(state->rtp, NULL)) { //start convert
			int ret = mkdir(lg_convert_base_path, //
					S_IRUSR | S_IWUSR | S_IXUSR | /* rwx */
					S_IRGRP | S_IWGRP | S_IXGRP | /* rwx */
					S_IROTH | S_IXOTH | S_IXOTH);
			if (ret == 0 || errno == EEXIST) {
				rtp_set_auto_play(state->rtp, false);
				rtp_set_is_looping(state->rtp, false);

				lg_is_converting = true;
				convert_snap_handler();

				snprintf(menu->name, 256, "StopConverting:%s",
						lg_convert_base_path);
				printf("start converting to %s\n", lg_convert_base_path);
			}
		} else if (rtp_is_recording(state->rtp, NULL)) { //stop record
			rtp_stop_recording(state->rtp);
			snprintf(menu->name, 8, "Record");
			printf("stop recording\n");
		} else if (!rtp_is_loading(state->rtp, NULL)) { //start record
			char dst[256];
			int last_id = get_last_id(PACKET_FOLDER_PATH);
			if (last_id >= 0) {
				snprintf(dst, 256, PACKET_FOLDER_PATH "/%d.rtp", last_id + 1);
				rtp_start_recording(state->rtp, dst);
				snprintf(menu->name, 256, "StopRecording:%s", dst);
				printf("start recording %s\n", dst);
			}
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

static void packet_menu_load_node_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED:
		break;
	case MENU_EVENT_DEACTIVATED:
		break;
	case MENU_EVENT_SELECTED:
		if (lg_is_converting) {
			//do nothing
		} else if (menu->marked) {
			rtp_stop_loading(state->rtp);
			printf("stop loading\n");
			menu->marked = false;
			menu->selected = false;
		} else if (!rtp_is_recording(state->rtp, NULL)
				&& !rtp_is_loading(state->rtp, NULL)) {
			char filepath[256];
			snprintf(filepath, 256, PACKET_FOLDER_PATH "/%s",
					(char*) menu->user_data);
			rtp_start_loading(state->rtp, filepath, true, true,
					(RTP_LOADING_CALLBACK) loading_callback, NULL);
			printf("start loading %s\n", filepath);
			menu->marked = true;
			menu->selected = false;

			snprintf(lg_convert_base_path, 256, VIDEO_FOLDER_PATH "/%s",
					(char*) menu->user_data);
			lg_convert_frame_num = 0;
		} else {
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

static void packet_menu_load_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_ACTIVATED:
		if (!rtp_is_loading(state->rtp, NULL)) {
			struct dirent *d;
			DIR *dir;

			dir = opendir(PACKET_FOLDER_PATH);
			if (dir != NULL) {
				while ((d = readdir(dir)) != 0) {
					if (d->d_name[0] != L'.') {
						int len = strlen(d->d_name);
						if (strncmp(d->d_name + len - 5, ".json", 5) == 0) {
							continue;
						}

						char *name = malloc(256);
						strncpy(name, d->d_name, 256);
						MENU_T *node_menu = menu_new(name,
								packet_menu_load_node_callback, name);
						menu_add_submenu(menu, node_menu, INT_MAX);
					}
				}
			}
		}
		break;
	case MENU_EVENT_DEACTIVATED:
		if (!rtp_is_loading(state->rtp, NULL)) {
			for (int idx = 0; menu->submenu[idx]; idx++) {
				menu_delete(&menu->submenu[idx]);
			}
		}
		break;
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

static void function_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		switch ((intptr_t) menu->user_data) {
		case 0: //snap
			menu->selected = false;
			{
				char dst[256];
				int last_id = get_last_id(STILL_FOLDER_PATH);
				if (last_id >= 0) {
					snprintf(dst, 256, STILL_FOLDER_PATH "/%d.jpeg",
							last_id + 1);
					state->plugin_host.snap(lg_resolution * 1024,
							lg_resolution * 512, RENDERING_MODE_EQUIRECTANGULAR,
							dst);
				}
			}
			break;
		default:
			break;
		}
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

static void stereo_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		lg_stereo_enabled = !(bool) menu->user_data;
		menu->user_data = (void*) lg_stereo_enabled;
		if (lg_stereo_enabled) {
			snprintf(menu->name, 8, "On");
		} else {
			snprintf(menu->name, 8, "Off");
		}
		{
			char cmd[256];
			snprintf(cmd, 256, "set_stereo %d", lg_stereo_enabled ? 1 : 0);
			state->plugin_host.send_command(cmd);
		}
		menu->selected = false;
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

static void play_speed_menu_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		for (int idx = 0; menu->parent->submenu[idx]; idx++) {
			menu->parent->submenu[idx]->marked = false;
		}
		menu->marked = true;
		menu->selected = false;
		{
			float value = (float) ((intptr_t) menu->user_data) / 100;
			char cmd[256];
			snprintf(cmd, 256, "set_play_speed %f", value);
			state->plugin_host.send_command(cmd);
		}
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

static void sync_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		lg_sync_enabled = !(bool) menu->user_data;
		menu->user_data = (void*) lg_sync_enabled;
		if (lg_sync_enabled) {
			snprintf(menu->name, 8, "On");
		} else {
			snprintf(menu->name, 8, "Off");
		}
		{
			char cmd[256];
			snprintf(cmd, 256, "set_conf_sync %d", lg_sync_enabled ? 1 : 0);
			state->plugin_host.send_command(cmd);
		}
		menu->selected = false;
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

static void resolution_menu_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		lg_resolution = (intptr_t) menu->user_data;
		for (int idx = 0; menu->parent->submenu[idx]; idx++) {
			menu->parent->submenu[idx]->marked = false;
		}
		menu->marked = true;
		menu->selected = false;
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

static void fov_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		if (1) {
			int value = (intptr_t) menu->user_data;
			if (value == -1 || value == 1) {
				value = state->plugin_host.get_fov() + value;
			}
			state->plugin_host.set_fov(MIN(MAX(value, 60), 120));
		}
		menu->selected = false;
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

static void calibration_menu_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		switch ((intptr_t) menu->user_data) {
		case CALIBRATION_CMD_SAVE:
			menu->selected = false;
			{
				char cmd[256];
				snprintf(cmd, 256, "save");
				state->plugin_host.send_command(cmd);
			}
			break;
		case CALIBRATION_CMD_VIEWER_COMPASS:
			if (1) {
				char cmd[256];
				snprintf(cmd, 256, "mpu9250.start_compass_calib");
				state->plugin_host.send_command(cmd);
				//snprintf(cmd, 256, "oculus_rift_dk2.start_compass_calib");
				//state->plugin_host.send_command(cmd);
			}
			break;
		case CALIBRATION_CMD_VEHICLE_COMPASS:
			if (1) { //this should be in plugin
				char cmd[256];
				snprintf(cmd, 256, "upstream.mpu9250.start_compass_calib");
				state->plugin_host.send_command(cmd);
			}
			break;
		default:
			break;
		}
		break;
	case MENU_EVENT_DESELECTED:
		switch ((intptr_t) menu->user_data) {
		case CALIBRATION_CMD_VIEWER_COMPASS:
			if (1) {
				char cmd[256];
				snprintf(cmd, 256, "mpu9250.stop_compass_calib");
				state->plugin_host.send_command(cmd);
				//snprintf(cmd, 256, "oculus_rift_dk2.stop_compass_calib");
				//state->plugin_host.send_command(cmd);
			}
			break;
		case CALIBRATION_CMD_VEHICLE_COMPASS:
			if (1) {
				char cmd[256];
				snprintf(cmd, 256, "upstream.mpu9250.stop_compass_calib");
				state->plugin_host.send_command(cmd);
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void image_params_calibration_menu_callback(struct _MENU_T *menu,
		enum MENU_EVENT event) {
	switch (event) {
	case MENU_EVENT_SELECTED:
		if (menu->user_data) {
			state->plugin_host.send_command((char*) menu->user_data);
			menu->selected = false;
		}
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

static void system_menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {
	int ret;
	switch (event) {
	case MENU_EVENT_SELECTED:
		if ((enum SYSTEM_CMD) menu->user_data == SYSTEM_CMD_SHUTDOWN) {
			ret = system("sudo shutdown now");
			exit(1);
		} else if ((enum SYSTEM_CMD) menu->user_data == SYSTEM_CMD_REBOOT) {
			ret = system("sudo reboot");
			exit(1);
		} else if ((enum SYSTEM_CMD) menu->user_data == SYSTEM_CMD_EXIT) {
			exit(1);
		}
		break;
	case MENU_EVENT_DESELECTED:
		break;
	default:
		break;
	}
}

static void menu_callback(struct _MENU_T *menu, enum MENU_EVENT event) {

}

void init_menu_handler(PICAM360CAPTURE_T *_state) {
	state = _state;

	state->menu = menu_new("Menu", menu_callback, NULL);
	state->menu->activated = true;

	MENU_T *menu = state->menu;
	{
		MENU_T *sub_menu = menu_add_submenu(menu,
				menu_new("Function", NULL, NULL), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new("Snap", function_menu_callback, (void*) 0), INT_MAX);
	}
	{
		MENU_T *sub_menu = menu_add_submenu(menu,
				menu_new("Config", NULL, NULL), INT_MAX);
		MENU_T *sync_menu = menu_add_submenu(sub_menu,
				menu_new("Sync", NULL, NULL), INT_MAX);
		{
			MENU_T *sub_menu = sync_menu;
			MENU_T *on_menu = menu_add_submenu(sub_menu,
					menu_new("On", sync_menu_callback, (void*) true), INT_MAX);
			on_menu->marked = true;
		}
		MENU_T *stereo_menu = menu_add_submenu(sub_menu,
				menu_new("Stereo", NULL, NULL), INT_MAX);
		{
			MENU_T *sub_menu = stereo_menu;
			MENU_T *off_menu = menu_add_submenu(sub_menu,
					menu_new("Off", stereo_menu_callback, (void*) false),
					INT_MAX);
			off_menu->marked = true;
		}
		MENU_T *resolution_menu = menu_add_submenu(sub_menu,
				menu_new("Resolution", NULL, NULL), INT_MAX);
		{
			MENU_T *sub_menu = resolution_menu;
			menu_add_submenu(sub_menu,
					menu_new("4K", resolution_menu_callback, (void*) 4),
					INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("3K", resolution_menu_callback, (void*) 3),
					INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("2K", resolution_menu_callback, (void*) 2),
					INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("1K", resolution_menu_callback, (void*) 1),
					INT_MAX);
			for (int idx = 0; sub_menu->submenu[idx]; idx++) {
				if (sub_menu->submenu[idx]->user_data
						== (void*) (intptr_t) lg_resolution) {
					sub_menu->submenu[idx]->marked = true;
				}
			}
		}
		MENU_T *play_speed_menu = menu_add_submenu(sub_menu,
				menu_new("PlaySpeed", NULL, NULL), INT_MAX);
		{
			MENU_T *sub_menu = play_speed_menu;
			menu_add_submenu(sub_menu,
					menu_new("x1/8", play_speed_menu_callback, (void*) 12),
					INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("x1/4", play_speed_menu_callback, (void*) 25),
					INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("x1/2", play_speed_menu_callback, (void*) 50),
					INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("x1", play_speed_menu_callback, (void*) 100),
					INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("x2", play_speed_menu_callback, (void*) 200),
					INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("x4", play_speed_menu_callback, (void*) 400),
					INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("x8", play_speed_menu_callback, (void*) 400),
					INT_MAX);
			for (int idx = 0; sub_menu->submenu[idx]; idx++) {
				int play_speed_percent = (int) (state->rtp_play_speed * 100);
				if (sub_menu->submenu[idx]->user_data
						== (void*) (intptr_t) play_speed_percent) {
					sub_menu->submenu[idx]->marked = true;
				}
			}
		}
		MENU_T *fov_menu = menu_add_submenu(sub_menu,
				menu_new("Fov", NULL, NULL), INT_MAX);
		{
			MENU_T *sub_menu = fov_menu;
			menu_add_submenu(sub_menu,
					menu_new("-", fov_menu_callback, (void*) -1), INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("+", fov_menu_callback, (void*) 1), INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("60", fov_menu_callback, (void*) 60), INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("90", fov_menu_callback, (void*) 90), INT_MAX);
			menu_add_submenu(sub_menu,
					menu_new("120", fov_menu_callback, (void*) 120), INT_MAX);
		}
	}
	{
		MENU_T *sub_menu = menu_add_submenu(menu,
				menu_new("Packet", NULL, NULL), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new("Record", packet_menu_record_callback, NULL), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new("Play", packet_menu_load_callback, NULL), INT_MAX);
	}
	{
		MENU_T *sub_menu = menu_add_submenu(menu,
				menu_new("Calibration", NULL, NULL), INT_MAX);
		MENU_T *image_params_menu = menu_add_submenu(sub_menu,
				menu_new("ImageParams", calibration_menu_callback,
						(void*) CALIBRATION_CMD_IMAGE_PARAMS), INT_MAX);
		{
			MENU_T *sub_menu = image_params_menu;
			{
				MENU_T *_sub_menu = menu_add_submenu(sub_menu,
						menu_new("brightness",
								image_params_calibration_menu_callback, NULL),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("-", image_params_calibration_menu_callback,
								(void*) ENDPOINT_DOMAIN "v4l2_capture.add_v4l2_ctl brightness=-1"),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("+", image_params_calibration_menu_callback,
								(void*) ENDPOINT_DOMAIN "v4l2_capture.add_v4l2_ctl brightness=1"),
						INT_MAX);
			}
			{
				MENU_T *_sub_menu = menu_add_submenu(sub_menu,
						menu_new("gamma",
								image_params_calibration_menu_callback, NULL),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("-", image_params_calibration_menu_callback,
								(void*) ENDPOINT_DOMAIN "v4l2_capture.add_v4l2_ctl gamma=-1"),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("+", image_params_calibration_menu_callback,
								(void*) ENDPOINT_DOMAIN "v4l2_capture.add_v4l2_ctl gamma=1"),
						INT_MAX);
			}
			{
				MENU_T *_sub_menu = menu_add_submenu(sub_menu,
						menu_new("contrast",
								image_params_calibration_menu_callback, NULL),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("-", image_params_calibration_menu_callback,
								(void*) ENDPOINT_DOMAIN "v4l2_capture.add_v4l2_ctl contrast=-1"),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("+", image_params_calibration_menu_callback,
								(void*) ENDPOINT_DOMAIN "v4l2_capture.add_v4l2_ctl contrast=1"),
						INT_MAX);
			}
			{
				MENU_T *_sub_menu = menu_add_submenu(sub_menu,
						menu_new("saturation",
								image_params_calibration_menu_callback, NULL),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("-", image_params_calibration_menu_callback,
								(void*) ENDPOINT_DOMAIN "v4l2_capture.add_v4l2_ctl saturation=-1"),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("+", image_params_calibration_menu_callback,
								(void*) ENDPOINT_DOMAIN "v4l2_capture.add_v4l2_ctl saturation=1"),
						INT_MAX);
			}
			{
				MENU_T *_sub_menu = menu_add_submenu(sub_menu,
						menu_new("sharpness",
								image_params_calibration_menu_callback, NULL),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("-", image_params_calibration_menu_callback,
								(void*) ENDPOINT_DOMAIN "v4l2_capture.add_v4l2_ctl sharpness=-1"),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("+", image_params_calibration_menu_callback,
								(void*) ENDPOINT_DOMAIN "v4l2_capture.add_v4l2_ctl sharpness=1"),
						INT_MAX);
			}
//			{
//				MENU_T *_sub_menu = menu_add_submenu(sub_menu, menu_new("white_balance_temperature_auto", image_params_calibration_menu_callback, NULL), INT_MAX);
//				menu_add_submenu(_sub_menu, menu_new("-", image_params_calibration_menu_callback, NULL), INT_MAX);
//				menu_add_submenu(_sub_menu, menu_new("+", image_params_calibration_menu_callback, NULL), INT_MAX);
//			}
//			{
//				MENU_T *_sub_menu = menu_add_submenu(sub_menu, menu_new("white_balance_temperature", image_params_calibration_menu_callback, NULL), INT_MAX);
//				menu_add_submenu(_sub_menu, menu_new("-", image_params_calibration_menu_callback, NULL), INT_MAX);
//				menu_add_submenu(_sub_menu, menu_new("+", image_params_calibration_menu_callback, NULL), INT_MAX);
//			}
//			{
//				MENU_T *_sub_menu = menu_add_submenu(sub_menu, menu_new("exposure_auto", image_params_calibration_menu_callback, NULL), INT_MAX);
//				menu_add_submenu(_sub_menu, menu_new("-", image_params_calibration_menu_callback, NULL, INT_MAX);
//				menu_add_submenu(_sub_menu, menu_new("+", image_params_calibration_menu_callback, NULL), INT_MAX);
//			}
			{
				MENU_T *_sub_menu = menu_add_submenu(sub_menu,
						menu_new("color_offset",
								image_params_calibration_menu_callback, NULL),
						INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("-", image_params_calibration_menu_callback,
								(void*) "add_color_offset -0.01"), INT_MAX);
				menu_add_submenu(_sub_menu,
						menu_new("+", image_params_calibration_menu_callback,
								(void*) "add_color_offset 0.01"), INT_MAX);
			}
		}
		menu_add_submenu(sub_menu,
				menu_new("ViewerCompass", calibration_menu_callback,
						(void*) CALIBRATION_CMD_VIEWER_COMPASS), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new("VehicleCompass", calibration_menu_callback,
						(void*) CALIBRATION_CMD_VEHICLE_COMPASS), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new("Save", calibration_menu_callback,
						(void*) CALIBRATION_CMD_SAVE), INT_MAX);
	}
	{
		MENU_T *sub_menu = menu_add_submenu(menu,
				menu_new("System", NULL, NULL), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new("Shutdown", system_menu_callback,
						(void*) SYSTEM_CMD_SHUTDOWN), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new("Reboot", system_menu_callback,
						(void*) SYSTEM_CMD_REBOOT), INT_MAX);
		menu_add_submenu(sub_menu,
				menu_new("Exit", system_menu_callback, (void*) SYSTEM_CMD_EXIT),
				INT_MAX);
	}
}
