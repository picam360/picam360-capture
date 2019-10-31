#pragma once

#include <pthread.h>
#include <stdbool.h>

typedef int (*REFERENCE_CALLBACK)(void *user_data);
typedef struct _REFERENCE_H {
	pthread_mutex_t mutex;
	int ref_count;
	int (*addref)(struct _REFERENCE_H *_this);
	int (*release)(struct _REFERENCE_H *_this);
	REFERENCE_CALLBACK callback;
	void *user_data;
}REFERENCE_H;

void reference_init(REFERENCE_H *obj, REFERENCE_CALLBACK callback, void *user_data);
