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

#include "stream_mixer.h"
#include "mrevent.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define BUFFER_NUM 4
struct _stream_mixer;
typedef struct _stream_mixer_input {
	VSTREAMER_T super;
	struct _stream_mixer *mixer;

	int id;
	bool run;
	pthread_t streaming_thread;

	int framecount;
	PICAM360_IMAGE_T *frame_buffers[BUFFER_NUM];

	struct _stream_mixer_input *next;
} stream_mixer_input;

typedef struct _stream_mixer_output {
	VSTREAMER_T super;
	struct _stream_mixer *mixer;

	int id;
	MREVENT_T frame_ready;

	struct _stream_mixer_output *next;
} stream_mixer_output;

typedef struct _stream_mixer {
	pthread_mutex_t mutex;
	int input_stream_last_id;
	int output_stream_last_id;
	stream_mixer_input *input_head;
	stream_mixer_output *output_head;
} stream_mixer;
static stream_mixer lg_mixer = { };

static void* input_streaming_thread_func(void *obj) {
	stream_mixer_input *_this = (stream_mixer_input*) obj;

	while (_this->run) {
		if (_this->super.pre_streamer == NULL) {
			usleep(100 * 1000);
			continue;
		}
		int ret;
		int num;
		PICAM360_IMAGE_T *image;
		ret = _this->super.pre_streamer->get_image(_this->super.pre_streamer,
				&image, &num, 100 * 1000);
		if (ret != 0) {
			continue;
		}
		bool new_frame = true;
		int cur = _this->framecount % BUFFER_NUM;
		if (_this->frame_buffers[cur] != NULL
				&& _this->frame_buffers[cur]->ref) {
			_this->frame_buffers[cur]->ref->release(
					_this->frame_buffers[cur]->ref);
			_this->frame_buffers[cur] = NULL;
		}
		_this->frame_buffers[cur] = image;
		pthread_mutex_lock(&_this->mixer->mutex);
		{
			_this->framecount++;

			for (stream_mixer_input *node = _this->mixer->input_head;
					node != NULL; node = node->next) {
				if (node->framecount < _this->framecount) {
					new_frame = false;
					break;
				}
			}
		}
		pthread_mutex_unlock(&_this->mixer->mutex);

		if (new_frame) {
			for (stream_mixer_output *node = _this->mixer->output_head;
					node != NULL; node = node->next) {
				mrevent_trigger(&node->frame_ready);
			}
		}
	}

	return NULL;
}

static void input_start(void *user_data) {
	stream_mixer_input *_this = (stream_mixer_input*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}

	_this->run = true;
	pthread_create(&_this->streaming_thread, NULL, input_streaming_thread_func,
			(void*) _this);
}

static void input_stop(void *user_data) {
	stream_mixer_input *_this = (stream_mixer_input*) user_data;

	if(_this->run){
		_this->run = false;
		pthread_join(_this->streaming_thread, NULL);
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static void input_release(void *obj) {
	stream_mixer_input *_this = (stream_mixer_input*) obj;

	if(_this->run){
		_this->super.stop(&_this->super);
	}

	pthread_mutex_lock(&_this->mixer->mutex);
	{
		stream_mixer_input **container_p = &_this->mixer->input_head;
		for (; (*container_p) != _this && (*container_p) != NULL; container_p =
				&(*container_p)->next) {
			//do nothing;
		}
		*container_p = (*container_p)->next;
	}
	pthread_mutex_unlock(&_this->mixer->mutex);

	free(obj);
}

static int input_get_image(void *obj, PICAM360_IMAGE_T **image_p, int *num_p,
		int wait_usec) {
	stream_mixer_input *_this = (stream_mixer_input*) obj;

	return -1;
}

static void output_start(void *user_data) {
	stream_mixer_output *_this = (stream_mixer_output*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}
}

static void output_stop(void *user_data) {
	stream_mixer_output *_this = (stream_mixer_output*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static void output_release(void *obj) {
	stream_mixer_output *_this = (stream_mixer_output*) obj;

	pthread_mutex_lock(&_this->mixer->mutex);
	{
		stream_mixer_output **container_p = &_this->mixer->output_head;
		for (; (*container_p) != _this && (*container_p) != NULL; container_p =
				&(*container_p)->next) {
			//do nothing;
		}
		*container_p = (*container_p)->next;
	}
	pthread_mutex_unlock(&_this->mixer->mutex);

	free(obj);
}

static int output_get_image(void *obj, PICAM360_IMAGE_T **image_p, int *num_p,
		int wait_usec) {
	stream_mixer_output *_this = (stream_mixer_output*) obj;

	int res = mrevent_wait(&_this->frame_ready, wait_usec);
	if (res != 0) {
		return -1;
	} else {
		mrevent_reset(&_this->frame_ready);
	}

	int max_framecount = _this->mixer->input_head->framecount;
	int min_framecount = _this->mixer->input_head->framecount;
	for (stream_mixer_input *node = _this->mixer->input_head; node != NULL;
			node = node->next) {
		min_framecount = MIN(min_framecount, node->framecount);
		max_framecount = MAX(min_framecount, node->framecount);
	}
	if(max_framecount - min_framecount > BUFFER_NUM){
		printf("sync error : %s\n", __FILE__);
	}
	int cur = (min_framecount - 1) % BUFFER_NUM;

	int input_num = 0;
	for (stream_mixer_input *node2 = _this->mixer->input_head; node2 != NULL;
			node2 = node2->next) {
		PICAM360_IMAGE_T *image = node2->frame_buffers[cur];
		if (image->ref) {
			image->ref->addref(image->ref);
		}
		image_p[input_num] = image;

		input_num++;
		if (input_num >= *num_p) {
			break;
		}
	}
	*num_p = input_num;

	return 0;
}

void stream_mixer_init() {
	pthread_mutex_init(&lg_mixer.mutex, NULL);
}

int stream_mixer_create_input(VSTREAMER_T **out_streamer) {
	VSTREAMER_T *streamer = (VSTREAMER_T*) malloc(sizeof(stream_mixer_input));
	memset(streamer, 0, sizeof(stream_mixer_input));
	strcpy(streamer->name, "MIXER_INPUT");
	streamer->release = input_release;
	streamer->start = input_start;
	streamer->stop = input_stop;
	streamer->get_image = input_get_image;
	streamer->user_data = streamer;

	stream_mixer_input *_private = (stream_mixer_input*) streamer;
	_private->mixer = &lg_mixer;
	_private->id = ++_private->mixer->input_stream_last_id;//id should start from 1

	pthread_mutex_lock(&_private->mixer->mutex);
	{
		stream_mixer_input **tail_p = &_private->mixer->input_head;
		for (; (*tail_p) != NULL; tail_p = &(*tail_p)->next) {
			//do nothing;
		}
		*tail_p = _private;
	}
	pthread_mutex_unlock(&_private->mixer->mutex);

	if (out_streamer) {
		*out_streamer = streamer;
	}
	return _private->id;
}

int stream_mixer_create_output(VSTREAMER_T **out_streamer) {
	VSTREAMER_T *streamer = (VSTREAMER_T*) malloc(sizeof(stream_mixer_output));
	memset(streamer, 0, sizeof(stream_mixer_output));
	strcpy(streamer->name, "MIXER_OUTPUT");
	streamer->release = output_release;
	streamer->start = output_start;
	streamer->stop = output_stop;
	streamer->get_image = output_get_image;
	streamer->user_data = streamer;

	stream_mixer_output *_private = (stream_mixer_output*) streamer;
	_private->mixer = &lg_mixer;
	_private->id = ++_private->mixer->output_stream_last_id;//id should start from 1
	mrevent_init(&_private->frame_ready);

	pthread_mutex_lock(&_private->mixer->mutex);
	{
		stream_mixer_output **tail_p = &_private->mixer->output_head;
		for (; (*tail_p) != NULL; tail_p = &(*tail_p)->next) {
			//do nothing;
		}
		*tail_p = _private;
	}
	pthread_mutex_unlock(&_private->mixer->mutex);

	if (out_streamer) {
		*out_streamer = streamer;
	}
	return _private->id;
}

int stream_mixer_get_input(int id, VSTREAMER_T **streamer) {
	*streamer = NULL;
	if(id < 0){
		stream_mixer_input *node = lg_mixer.input_head;
		if(node == NULL){
			return -1;
		}else{
			*streamer = &node->super;
			return node->id;
		}
	}
	for (stream_mixer_input *node = lg_mixer.input_head; node != NULL;
			node = node->next) {
		if(node->id == id) {
			*streamer = &node->super;
			return node->id;
		}
	}
	return -1;
}

int stream_mixer_get_output(int id, VSTREAMER_T **streamer) {
	*streamer = NULL;
	if(id < 0){
		stream_mixer_output *node = lg_mixer.output_head;
		if(node == NULL){
			return -1;
		}else{
			*streamer = &node->super;
			return node->id;
		}
	}
	for (stream_mixer_output *node = lg_mixer.output_head; node != NULL;
			node = node->next) {
		if(node->id == id) {
			*streamer = &node->super;
			return node->id;
		}
	}
	return -1;
}
