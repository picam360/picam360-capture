#pragma once

#include "picam360_capture_plugin.h"

void stream_mixer_init();
void stream_mixer_create_input(VSTREAMER_T **streamer);
void stream_mixer_create_output(VSTREAMER_T **streamer);
