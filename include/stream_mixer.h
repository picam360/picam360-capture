#pragma once

#include "picam360_capture_plugin.h"

void stream_mixer_init();
int stream_mixer_create_input(VSTREAMER_T **streamer);
int stream_mixer_create_output(VSTREAMER_T **streamer);
