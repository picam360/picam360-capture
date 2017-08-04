#include "mrevent.h"
#include <sys/time.h>
#include <errno.h>

void mrevent_init(MREVENT_T *ev) {
	pthread_mutex_init(&ev->mutex, 0);
	pthread_cond_init(&ev->cond, 0);
	ev->triggered = false;
}

void mrevent_trigger(MREVENT_T *ev) {
	pthread_mutex_lock(&ev->mutex); // this is for avoiding infinity mrevent_wait
	ev->triggered = true;
	pthread_cond_broadcast(&ev->cond);
	pthread_mutex_unlock(&ev->mutex);
}

void mrevent_reset(MREVENT_T *ev) { // this is for avoiding through mrevent_wait
	pthread_mutex_lock(&ev->mutex);
	ev->triggered = false;
	pthread_mutex_unlock(&ev->mutex);
}

int mrevent_wait(MREVENT_T *ev, long usec) {
	int retcode = 0;

	pthread_mutex_lock(&ev->mutex);
	if (ev->triggered) {
		//do nothing
	} else if (usec > 0) {
		long sec;
		struct timeval now;
		struct timespec timeout;
		gettimeofday(&now, NULL);
		usec += now.tv_usec;

		sec = usec / 1000000;
		usec = usec % 1000000;
		timeout.tv_sec = now.tv_sec + sec;
		timeout.tv_nsec = usec * 1000;
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
