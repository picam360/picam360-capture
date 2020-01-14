#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <limits.h>
#include <sys/time.h>
#include <list>
#include <dlfcn.h>
#include <assert.h>
#include <linux/videodev2.h>

#include "mjpeg_omx_encoder.h"
#include "mrevent.h"

#include "omxjpeg.h"

#define PLUGIN_NAME "mjpeg_omx_encoder"
#define ENCODER_NAME "mjpeg_encoder"

#define TIMEOUT_MS 2000

#include "user-vcsm.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

//libomxjpeg.a
#define PFN(fnc_name) lg_##fnc_name
#define PFN_DEFINE(ret_type, fnc_name, args) \
		typedef ret_type (*pfn_##fnc_name) args ;\
		static pfn_##fnc_name lg_##fnc_name = NULL
#define PFN_LOAD(handle, fnc_name)\
		lg_##fnc_name = (pfn_##fnc_name)omxjpeg_##fnc_name;\
		assert(lg_##fnc_name)

static bool init_pfn();
static bool lg_is_init_pfn = init_pfn();

PFN_DEFINE(void, jpeg_CreateCompress,
		(j_compress_ptr cinfo, int version, size_t structsize));
PFN_DEFINE(void, jpeg_destroy_compress, (j_compress_ptr cinfo));
PFN_DEFINE(void, jpeg_suppress_tables,
		(j_compress_ptr cinfo, boolean suppress));
PFN_DEFINE(void, jpeg_mem_dest,
		(j_compress_ptr cinfo, unsigned char ** outbuffer,unsigned long * outsize));
PFN_DEFINE(void, jpeg_set_defaults, (j_compress_ptr cinfo));
PFN_DEFINE(void, jpeg_set_quality,
		(j_compress_ptr cinfo, int quality, boolean force_baseline));
PFN_DEFINE(void, jpeg_set_hardware_acceleration_parameters_enc,
		(j_compress_ptr cinfo, boolean hw_acceleration, unsigned int defaultBuffSize, unsigned int defaultWidth, unsigned int defaultHeight ));
PFN_DEFINE(JDIMENSION, jpeg_write_raw_data,
		(j_compress_ptr cinfo, JSAMPIMAGE data, JDIMENSION num_lines));
PFN_DEFINE(void, jpeg_start_compress,
		(j_compress_ptr cinfo, boolean write_all_tables));
PFN_DEFINE(void, jpeg_finish_compress, (j_compress_ptr cinfo));
PFN_DEFINE(struct jpeg_error_mgr*, jpeg_std_error,
		(struct jpeg_error_mgr * err));

static bool init_pfn() {
	if (lg_is_init_pfn) {
		return true;
	}
	{
		PFN_LOAD(handle, jpeg_CreateCompress);
		PFN_LOAD(handle, jpeg_destroy_compress);
		PFN_LOAD(handle, jpeg_suppress_tables);
		PFN_LOAD(handle, jpeg_mem_dest);
		PFN_LOAD(handle, jpeg_set_defaults);
		PFN_LOAD(handle, jpeg_set_quality);
		PFN_LOAD(handle, jpeg_set_hardware_acceleration_parameters_enc);
		PFN_LOAD(handle, jpeg_write_raw_data);
		PFN_LOAD(handle, jpeg_start_compress);
		PFN_LOAD(handle, jpeg_finish_compress);
		PFN_LOAD(handle, jpeg_std_error);
	}
	return true;
}

static PLUGIN_HOST_T *lg_plugin_host = NULL;

#define BUFFER_NUM 2

static int lg_quality = 85; //google recomended

typedef struct _mjpeg_omx_encoder_private {
	VSTREAMER_T super;

	bool run;
	int framecount;

	MREVENT_T frame_ready;
	struct jpeg_compress_struct cinfos[BUFFER_NUM];
	PICAM360_IMAGE_T frame_buffers[BUFFER_NUM];
	unsigned char *outbuffers[BUFFER_NUM];
	unsigned long outsizes[BUFFER_NUM];
	pthread_t streaming_thread;
} mjpeg_omx_encoder_private;

static void encode(mjpeg_omx_encoder_private *_this, PICAM360_IMAGE_T *image) {
	int cur = _this->framecount % BUFFER_NUM;
	struct jpeg_compress_struct &cinfo = _this->cinfos[cur];

	unsigned char *&outbuffer = _this->outbuffers[cur];
	unsigned long &outsize = _this->outsizes[cur];
	outsize = image->width[0] * image->height[0] * 3 / 2;

	PICAM360_IMAGE_T *frame = &_this->frame_buffers[cur];
	frame->timestamp = image->timestamp;
	frame->meta = image->meta;
	frame->meta_size = image->meta_size;
	memcpy(frame->img_type, "JPEG", 4);
	frame->mem_type = PICAM360_MEMORY_TYPE_PROCESS;

	struct jpeg_error_mgr jerr;
	if (cinfo.fd == 0) {
		memset(&cinfo, 0, sizeof(cinfo));
		memset(&jerr, 0, sizeof(jerr));
		cinfo.err = PFN(jpeg_std_error)(&jerr); //local address

		PFN(jpeg_CreateCompress)(&cinfo, JPEG_LIB_VERSION, sizeof(cinfo));
		PFN(jpeg_suppress_tables)(&cinfo, TRUE);

		outbuffer = (uint8_t*) malloc(outsize);

		int quality = lg_quality;
		PFN(jpeg_mem_dest)(&cinfo, &outbuffer, &outsize);

		cinfo.fd = image->id[0];
		cinfo.IsVendorbuf = TRUE;

		cinfo.raw_data_in = TRUE;
		cinfo.in_color_space = JCS_YCbCr;
		cinfo.image_width = image->width[0];
		cinfo.image_height = image->height[0];
		PFN(jpeg_set_defaults)(&cinfo);
		PFN(jpeg_set_quality)(&cinfo, quality, TRUE);
		PFN(jpeg_set_hardware_acceleration_parameters_enc)(&cinfo, TRUE,
				outsize, 0, 0);
	} else {
		memset(&jerr, 0, sizeof(jerr));
		cinfo.err = PFN(jpeg_std_error)(&jerr); //local address
		cinfo.dest->next_output_byte = outbuffer;
		cinfo.dest->free_in_buffer = outsize;
	}

	struct egl_image_brcm_vcsm_info *vcsm_infop =
			(struct egl_image_brcm_vcsm_info*) image->pixels[0];
	//VCSM_CACHE_TYPE_T cache_type;
	//unsigned char *buffer = (unsigned char*) vcsm_lock_cache(
	//		vcsm_infop->vcsm_handle, VCSM_CACHE_TYPE_HOST, &cache_type);
	unsigned char *buffer = (unsigned char*) vcsm_lock(
			vcsm_infop->vcsm_handle);
	if (!buffer) {
		printf("Failed to lock VCSM buffer for handle %d\n",
				vcsm_infop->vcsm_handle);
		return;
	}
	cinfo.client_data = buffer;

	PFN(jpeg_start_compress)(&cinfo, 0);

	if (cinfo.err->msg_code) {
		char err_string[256];
		cinfo.err->format_message((j_common_ptr) &cinfo, err_string);
		printf("%s", err_string);

		vcsm_unlock_ptr(buffer);
		return;
	}

	PFN(jpeg_write_raw_data)(&cinfo, NULL, 0);
	PFN(jpeg_finish_compress)(&cinfo);

	vcsm_unlock_ptr(buffer);

	frame->pixels[0] = outbuffer;
	frame->width[0] = outsize;
	frame->stride[0] = outsize;
	frame->height[0] = 1;

	_this->framecount++;
	mrevent_trigger(&_this->frame_ready);
}

static void* streaming_thread_fnc(void *obj) {
	pthread_setname_np(pthread_self(), "mjpeg_encoder");
	mjpeg_omx_encoder_private *_this = (mjpeg_omx_encoder_private*) obj;

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
		if (num != 1 || memcmp(image->img_type, "RGB\0", 4) != 0
				|| image->mem_type != PICAM360_MEMORY_TYPE_EGL) {
			printf("%s : something wrong!\n", __FILE__);
			continue;
		}

		encode(_this, image);

		if (image->ref) {
			image->ref->release(image->ref);
		}

		{
			static int count = 0;
			static float elapsed_sec = 0;
			float _elapsed_sec;
			struct timeval diff;

			struct timeval now;
			gettimeofday(&now, NULL);

			timersub(&now, &image->timestamp, &diff);
			_elapsed_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
			if ((count % 200) == 0) {
				printf("mjpeg encode : %f\n", _elapsed_sec);
			}
			count++;
		}
	}

	return NULL;
}

static void start(void *user_data) {
	mjpeg_omx_encoder_private *_this = (mjpeg_omx_encoder_private*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}

	_this->run = true;
	pthread_create(&_this->streaming_thread, NULL, streaming_thread_fnc,
			user_data);
}

static void stop(void *user_data) {
	mjpeg_omx_encoder_private *_this = (mjpeg_omx_encoder_private*) user_data;

	if (_this->run) {
		_this->run = false;
		pthread_join(_this->streaming_thread, NULL);
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static int get_image(void *obj, PICAM360_IMAGE_T **image_p, int *num_p,
		int wait_usec) {
	mjpeg_omx_encoder_private *_this = (mjpeg_omx_encoder_private*) obj;

	int res = mrevent_wait(&_this->frame_ready, wait_usec);
	if (res != 0) {
		return -1;
	} else {
		mrevent_reset(&_this->frame_ready);
	}

	int cur = (_this->framecount - 1) % BUFFER_NUM;

	*image_p = &_this->frame_buffers[cur];
	*num_p = 1;

	return 0;
}

static void release(void *obj) {
	mjpeg_omx_encoder_private *_this = (mjpeg_omx_encoder_private*) obj;

	if (_this->run) {
		_this->super.stop(&_this->super);
	}

	int ret;
	for (int i = 0; i < BUFFER_NUM; i++) {
		if (_this->cinfos[i].fd > 0) {
			_this->cinfos[i].fd = 0;
			PFN(jpeg_destroy_compress)(&_this->cinfos[i]);
		}
		if (_this->outbuffers[i]) {
			free(_this->outbuffers[i]);
			_this->outbuffers[i] = NULL;
		}
	}

	free(obj);
}

static int set_param(void *obj, const char *param, const char *value_str) {
	mjpeg_omx_encoder_private *_this = (mjpeg_omx_encoder_private*) obj;
	return 0;
}

static int get_param(void *obj, const char *param, char *value_str, int size) {
	mjpeg_omx_encoder_private *_this = (mjpeg_omx_encoder_private*) obj;
	if (strcmp(param, "alignment") == 0) {
		sprintf(value_str, "%d", 256);
	}
	return 0;
}

static void create_encoder_mjpeg(void *user_data,
		VSTREAMER_T **output_encoder) {
	VSTREAMER_T *encoder = (VSTREAMER_T*) malloc(
			sizeof(mjpeg_omx_encoder_private));
	memset(encoder, 0, sizeof(mjpeg_omx_encoder_private));
	strcpy(encoder->name, ENCODER_NAME);
	encoder->release = release;
	encoder->start = start;
	encoder->stop = stop;
	encoder->set_param = set_param;
	encoder->get_param = get_param;
	encoder->get_image = get_image;
	encoder->user_data = encoder;

	mjpeg_omx_encoder_private *_private = (mjpeg_omx_encoder_private*) encoder;
	mrevent_init(&_private->frame_ready);

	if (output_encoder) {
		*output_encoder = encoder;
	}
}

static int command_handler(void *user_data, const char *_buff) {
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id,
		uint32_t event_id) {
}

static void init_options(void *user_data, json_t *_options) {
	PLUGIN_T *plugin = (PLUGIN_T*) user_data;
	json_t *options = json_object_get(_options, PLUGIN_NAME);
	if (options == NULL) {
		return;
	}
	lg_quality = json_number_value(json_object_get(options, "quality"));
}

static void save_options(void *user_data, json_t *_options) {
	json_t *options = json_object();
	json_object_set_new(_options, PLUGIN_NAME, options);

	json_object_set_new(options, "quality", json_real(lg_quality));
}

void create_mjpeg_encoder_plugin(PLUGIN_HOST_T *plugin_host,
		PLUGIN_T **_plugin) {
	lg_plugin_host = plugin_host;

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(PLUGIN_T));
		memset(plugin, 0, sizeof(PLUGIN_T));
		strcpy(plugin->name, PLUGIN_NAME);
		plugin->release = release;
		plugin->command_handler = command_handler;
		plugin->event_handler = event_handler;
		plugin->init_options = init_options;
		plugin->save_options = save_options;
		plugin->get_info = NULL;
		plugin->user_data = plugin;

		*_plugin = plugin;
	}
	{
		VSTREAMER_FACTORY_T *factory = (VSTREAMER_FACTORY_T*) malloc(
				sizeof(VSTREAMER_FACTORY_T));
		memset(factory, 0, sizeof(VSTREAMER_FACTORY_T));
		strcpy(factory->name, ENCODER_NAME);
		factory->release = release;
		factory->create_vstreamer = create_encoder_mjpeg;
		factory->user_data = factory;

		lg_plugin_host->add_vstreamer_factory(factory);
	}
}
