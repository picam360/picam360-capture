#pragma once

#include "picam360_capture_plugin.h"

enum STREAM_MIXER_EVENT{
	STREAM_MIXER_EVENT_ALL_INPUT_RELEASED, STREAM_MIXER_EVENT_ALL_OUTPUT_RELEASED,
};
typedef struct _STREAM_MIXER_T {
	char name[64];
	void (*release)(void *user_data);

	int (*create_input)(void *user_data, VSTREAMER_T **streamer);
	int (*create_output)(void *user_data, VSTREAMER_T **streamer);
	int (*get_input)(void *user_data, int id, VSTREAMER_T **streamer);
	int (*get_output)(void *user_data, int id, VSTREAMER_T **streamer);

	int (*event_callback)(void *user_data, enum STREAM_MIXER_EVENT event);

	void *user_data;
} STREAM_MIXER_T;

void create_stream_mixer(STREAM_MIXER_T **p);
