#pragma once

#include "picam360_capture_plugin.h"

void stream_mixer_init();
int stream_mixer_create_input(VSTREAMER_T **streamer);
int stream_mixer_create_output(VSTREAMER_T **streamer);

int stream_mixer_get_input(int id, VSTREAMER_T **streamer);
int stream_mixer_get_output(int id, VSTREAMER_T **streamer);
