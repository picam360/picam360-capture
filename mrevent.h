#ifndef _MREVENT_H
#define _MREVENT_H

#include <pthread.h>
#include <stdbool.h>

typedef struct {
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	bool triggered;
} MREVENT_T;

void mrevent_init(MREVENT_T *ev);

void mrevent_trigger(MREVENT_T *ev);

void mrevent_reset(MREVENT_T *ev);

int mrevent_wait(MREVENT_T *ev, long usec);

#endif
