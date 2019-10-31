#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <assert.h>
#include <sys/time.h>
#include <limits.h>
#if __linux
#include <sys/prctl.h>
#endif

#include "reference.h"

static int addref(struct _REFERENCE_H *_this) {
	int ret;
	pthread_mutex_lock(&_this->mutex);
	{
		ret = ++_this->ref_count;
	}
	pthread_mutex_unlock(&_this->mutex);
	return ret;
}
static int release(struct _REFERENCE_H *_this) {
	int ret;
	pthread_mutex_lock(&_this->mutex);
	{
		ret = --_this->ref_count;
		if(ret == 0 && _this->callback){
			_this->callback(_this->user_data);
		}
	}
	pthread_mutex_unlock(&_this->mutex);
	if(ret == 0){
		free(_this);
	}
	return ret;
}
void create_reference(REFERENCE_H **out_obj, REFERENCE_CALLBACK callback, void *user_data) {
	REFERENCE_H *obj = (REFERENCE_H*) malloc(sizeof(REFERENCE_H));
	obj->ref_count = 0;
	obj->addref = addref;
	obj->release = release;
	pthread_mutex_init(&obj->mutex, NULL);

	obj->callback = callback;
	obj->user_data = user_data;

	if (out_obj) {
		*out_obj = obj;
	}
}
