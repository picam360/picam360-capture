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
struct _stream_mixer_private;
typedef struct _stream_mixer_input {
	VSTREAMER_T super;
	struct _stream_mixer_private *mixer;

	int id;
	bool run;
	pthread_t streaming_thread;

	int framecount;
	PICAM360_IMAGE_T *frame_buffers[BUFFER_NUM][MAX_CAM_NUM + 1];

	struct _stream_mixer_input *next;
} stream_mixer_input;

typedef struct _stream_mixer_output {
	VSTREAMER_T super;
	struct _stream_mixer_private *mixer;

	int id;
	MREVENT_T frame_ready;

	struct _stream_mixer_output *next;
} stream_mixer_output;

typedef struct _stream_mixer_private {
	STREAM_MIXER_T super;

	pthread_mutex_t mutex;
	int input_stream_last_id;
	int output_stream_last_id;
	stream_mixer_input *input_head;
	stream_mixer_output *output_head;
} stream_mixer_private;

static void* input_streaming_thread_func(void *obj) {
	stream_mixer_input *_this = (stream_mixer_input*) obj;

	while (_this->run) {
		if (_this->super.pre_streamer == NULL) {
			usleep(100 * 1000);
			continue;
		}
		int ret;
		int num = MAX_CAM_NUM;
		PICAM360_IMAGE_T *images[MAX_CAM_NUM];
		ret = _this->super.pre_streamer->get_image(_this->super.pre_streamer,
				images, &num, 100 * 1000);
		if (ret != 0) {
			continue;
		}
		bool new_frame = true;
		int cur = _this->framecount % BUFFER_NUM;
		for (int i = 0; i < num; i++) {
			if (_this->frame_buffers[cur][i] != NULL
					&& _this->frame_buffers[cur][i]->ref) {
				_this->frame_buffers[cur][i]->ref->release(
						_this->frame_buffers[cur][i]->ref);
				_this->frame_buffers[cur][i] = NULL;
			}
			_this->frame_buffers[cur][i] = images[i];
		}
		_this->frame_buffers[cur][num] = NULL;
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

	if (_this->run) {
		_this->run = false;
		pthread_join(_this->streaming_thread, NULL);
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static void input_release(void *obj) {
	stream_mixer_input *_this = (stream_mixer_input*) obj;

	if (_this->run) {
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

	if (_this->mixer->input_head == NULL
			&& _this->mixer->super.event_callback) {
		_this->mixer->super.event_callback(_this->mixer,
				STREAM_MIXER_EVENT_ALL_INPUT_RELEASED);
	}

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

	if (_this->mixer->output_head == NULL
			&& _this->mixer->super.event_callback) {
		_this->mixer->super.event_callback(_this->mixer,
				STREAM_MIXER_EVENT_ALL_OUTPUT_RELEASED);
	}

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
	if (max_framecount - min_framecount > BUFFER_NUM) {
		printf("sync error : %s\n", __FILE__);
	}
	int cur = (min_framecount - 1) % BUFFER_NUM;

	int num = 0;
	for (stream_mixer_input *node2 = _this->mixer->input_head; node2 != NULL;
			node2 = node2->next) {
		for (int i = 0; num < *num_p && i < MAX_CAM_NUM; i++) {
			PICAM360_IMAGE_T *image = node2->frame_buffers[cur][i];
			if (image == NULL) {
				break;
			}
			if (image->ref) {
				image->ref->addref(image->ref);
			}
			image_p[num] = image;

			num++;
		}
	}
	*num_p = num;

	return 0;
}

static int create_input(void *obj, VSTREAMER_T **out_streamer) {
	stream_mixer_private *mixer = (stream_mixer_private*) obj;
	VSTREAMER_T *streamer = (VSTREAMER_T*) malloc(sizeof(stream_mixer_input));
	memset(streamer, 0, sizeof(stream_mixer_input));
	strcpy(streamer->name, "MIXER_INPUT");
	streamer->release = input_release;
	streamer->start = input_start;
	streamer->stop = input_stop;
	streamer->get_image = input_get_image;
	streamer->user_data = streamer;

	stream_mixer_input *_private = (stream_mixer_input*) streamer;
	_private->mixer = mixer;
	_private->id = ++_private->mixer->input_stream_last_id; //id should start from 1

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

static int create_output(void *obj, VSTREAMER_T **out_streamer) {
	stream_mixer_private *mixer = (stream_mixer_private*) obj;
	VSTREAMER_T *streamer = (VSTREAMER_T*) malloc(sizeof(stream_mixer_output));
	memset(streamer, 0, sizeof(stream_mixer_output));
	strcpy(streamer->name, "MIXER_OUTPUT");
	streamer->release = output_release;
	streamer->start = output_start;
	streamer->stop = output_stop;
	streamer->get_image = output_get_image;
	streamer->user_data = streamer;

	stream_mixer_output *_private = (stream_mixer_output*) streamer;
	_private->mixer = mixer;
	_private->id = ++_private->mixer->output_stream_last_id; //id should start from 1
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

static int get_input(void *obj, int id, VSTREAMER_T **streamer) {
	stream_mixer_private *mixer = (stream_mixer_private*) obj;
	*streamer = NULL;
	if (id <= 0) {
		stream_mixer_input *node = mixer->input_head;
		if (node == NULL) {
			return 0;
		} else {
			*streamer = &node->super;
			return node->id;
		}
	}
	for (stream_mixer_input *node = mixer->input_head; node != NULL;
			node = node->next) {
		if (node->id == id) {
			*streamer = &node->super;
			return node->id;
		}
	}
	return 0;
}

static int get_output(void *obj, int id, VSTREAMER_T **streamer) {
	stream_mixer_private *mixer = (stream_mixer_private*) obj;
	*streamer = NULL;
	if (id <= 0) {
		stream_mixer_output *node = mixer->output_head;
		if (node == NULL) {
			return 0;
		} else {
			*streamer = &node->super;
			return node->id;
		}
	}
	for (stream_mixer_output *node = mixer->output_head; node != NULL; node =
			node->next) {
		if (node->id == id) {
			*streamer = &node->super;
			return node->id;
		}
	}
	return 0;
}

static void mixer_release(void *obj) {
	stream_mixer_private *_this = (stream_mixer_private*) obj;

	free(obj);
}

void create_stream_mixer(STREAM_MIXER_T **p) {
	STREAM_MIXER_T *obj = (STREAM_MIXER_T*) malloc(
			sizeof(stream_mixer_private));
	memset(obj, 0, sizeof(stream_mixer_private));
	obj->release = mixer_release;
	obj->create_input = create_input;
	obj->create_output = create_output;
	obj->get_input = get_input;
	obj->get_output = get_output;
	obj->user_data = obj;

	stream_mixer_private *private = (stream_mixer_private*) obj;
	pthread_mutex_init(&private->mutex, NULL);

	*p = obj;
}
