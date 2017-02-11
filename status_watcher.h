#pragma once


void* status_watch(void* arg);

typedef void (*ATTITUDE_CALLBACK)(float *quatanion);
void set_attitude_callback(ATTITUDE_CALLBACK callback);
