#include "mrevent.h"

void mrevent_init(MREVENT_T *ev) {
	pthread_mutex_init(&ev->mutex, 0);
	pthread_cond_init(&ev->cond, 0);
	ev->triggered = false;
}

void mrevent_trigger(MREVENT_T *ev) {
	pthread_mutex_lock(&ev->mutex);
	ev->triggered = true;
	pthread_cond_broadcast(&ev->cond);
	pthread_mutex_unlock(&ev->mutex);
}

void mrevent_reset(MREVENT_T *ev) {
	pthread_mutex_lock(&ev->mutex);
	ev->triggered = false;
	pthread_mutex_unlock(&ev->mutex);
}

int mrevent_wait(MREVENT_T *ev, long usec) {
	int retcode = 0;

	pthread_mutex_lock(&ev->mutex);
	if (usec > 0) {
		struct timeval now;
		struct timespec timeout;
		gettimeofday(&now);
		timeout.tv_sec = now.tv_sec;
		timeout.tv_nsec = (now.tv_usec + usec) * 1000;
		while (!ev->triggered && retcode != ETIMEDOUT) {
			retcode = pthread_cond_timedwait(&ev->cond, &ev->mutex, &timeout);
		}
	} else {
		while (!ev->triggered) {
			pthread_cond_wait(&ev->cond, &ev->mutex);
		}
	}
	pthread_mutex_unlock(&ev->mutex);

	return retcode;
}
