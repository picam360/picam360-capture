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

#include "mjpeg_omx_decoder.h"
#include "mrevent.h"

#include "omxjpeg.h"

#define PLUGIN_NAME "mjpeg_omx_decoder"
#define DECODER_NAME "mjpeg_omx_decoder"

#define TIMEOUT_MS 2000

#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#include "jpeglib.h"

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

PFN_DEFINE(void, jpeg_CreateDecompress,
		(j_decompress_ptr cinfo, int version, size_t structsize));
PFN_DEFINE(void, jpeg_destroy_decompress, (j_decompress_ptr cinfo));
PFN_DEFINE(void, jpeg_mem_src,
		(j_decompress_ptr cinfo, unsigned char * inbuffer, unsigned long insize));
PFN_DEFINE(int, jpeg_read_header,
		(j_decompress_ptr cinfo, boolean require_image));
PFN_DEFINE(boolean, jpeg_start_decompress, (j_decompress_ptr cinfo));
PFN_DEFINE(JDIMENSION, jpeg_read_raw_data,
		(j_decompress_ptr cinfo, JSAMPIMAGE data, JDIMENSION max_lines));
PFN_DEFINE(boolean, jpeg_finish_decompress, (j_decompress_ptr cinfo));
PFN_DEFINE(struct jpeg_error_mgr*, jpeg_std_error,
		(struct jpeg_error_mgr * err));

static bool init_pfn() {
	if (lg_is_init_pfn) {
		return true;
	}
	{
		PFN_LOAD(handle, jpeg_CreateDecompress);
		PFN_LOAD(handle, jpeg_destroy_decompress);
		PFN_LOAD(handle, jpeg_mem_src);
		PFN_LOAD(handle, jpeg_read_header);
		PFN_LOAD(handle, jpeg_start_decompress);
		PFN_LOAD(handle, jpeg_read_raw_data);
		PFN_LOAD(handle, jpeg_finish_decompress);
		PFN_LOAD(handle, jpeg_std_error);
	}
	return true;
}

static PLUGIN_HOST_T *lg_plugin_host = NULL;

#define BUFFER_NUM 8
typedef struct _mjpeg_omx_decoder_private {
	VSTREAMER_T super;

	bool run;
	int framecount;
	int image_unit;
	float fps;
	int frameskip;
	MREVENT_T frame_ready;
	struct jpeg_decompress_struct cinfos[BUFFER_NUM];
	PICAM360_IMAGE_T frame_buffers[BUFFER_NUM];
	uint8_t xmp_buffers[BUFFER_NUM][RTP_MAXPAYLOADSIZE];
	pthread_t streaming_thread;
} mjpeg_omx_decoder_private;

static void decode(mjpeg_omx_decoder_private *_this, PICAM360_IMAGE_T *image) {
	uint8_t *in_buf = image->pixels[0];
	int in_buf_size = image->stride[0];

	if (in_buf_size < 10000) {
		//fail safe
		printf("my_fill_buffer_done : too small image size : %d\n",
				in_buf_size);
		return;
	}

	int image_width = 0;
	int image_height = 0;
	uint8_t *data = image->pixels[0];
	int data_len = image->stride[0];
	for (int i = 0; i < data_len; i++) {
		if (data[i] == 0xFF && data[i + 1] == 0xC0) {
			image_height += data[i + 5] << 8;
			image_height += data[i + 6] << 0;

			image_width += data[i + 7] << 8;
			image_width += data[i + 8] << 0;
			break;
		}
	}

	uint32_t pixel_format = 0;
	int cur = _this->framecount % BUFFER_NUM;

	struct jpeg_decompress_struct *cinfop = &_this->cinfos[cur];
	struct jpeg_error_mgr jerr;
	bool reuse = false;
	if (cinfop->fd == 0 || (_this->framecount % (10 * BUFFER_NUM + 1)) == 0) {
	} else {
		if (image_width == cinfop->image_width
				&& image_height == cinfop->image_height) {
			reuse = true;
		}
	}
	if (cinfop->fd > 0 && !reuse) {
		//release
		eglDestroyImageKHR(lg_plugin_host->get_display(),
				(EGLImageKHR) cinfop->client_data);
		glDeleteTextures(1, (GLuint*) &cinfop->fd);
		PFN(jpeg_destroy_decompress)(cinfop);
	}

	if (!reuse) {
		memset(cinfop, 0, sizeof(*cinfop));
		memset(&jerr, 0, sizeof(jerr));
		cinfop->err = PFN(jpeg_std_error)(&jerr); //local address

		PFN(jpeg_CreateDecompress)(cinfop, JPEG_LIB_VERSION,
				sizeof(*cinfop));
		PFN(jpeg_mem_src)(cinfop, in_buf, in_buf_size);

		{
			glGenTextures(1, (GLuint*) &cinfop->fd);

			glBindTexture(GL_TEXTURE_2D, (GLuint) cinfop->fd);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_width, 0,
					GL_RGBA, GL_UNSIGNED_BYTE, NULL);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

			/* Create EGL Image */
			cinfop->client_data = (void*) eglCreateImageKHR(
					lg_plugin_host->get_display(),
					lg_plugin_host->get_context(), EGL_GL_TEXTURE_2D_KHR,
					(EGLClientBuffer) cinfop->fd, 0);

			if (cinfop->client_data == EGL_NO_IMAGE_KHR) {
				printf("eglCreateImageKHR failed.\n");
				exit(1);
			}
		}
	} else {
		memset(&jerr, 0, sizeof(jerr));
		cinfop->err = PFN(jpeg_std_error)(&jerr); //local address
		cinfop->src->next_input_byte = in_buf;
		cinfop->src->bytes_in_buffer = in_buf_size;
	}
	PFN(jpeg_read_header)(cinfop, TRUE);

	cinfop->IsVendorbuf = TRUE;
	cinfop->out_color_space = JCS_YCbCr;

	PFN(jpeg_start_decompress)(cinfop);
	PFN(jpeg_read_raw_data)(cinfop, NULL,
			cinfop->comp_info[0].v_samp_factor * DCTSIZE);
	PFN(jpeg_finish_decompress)(cinfop);

	{ //meta
		_this->frame_buffers[cur].timestamp = image->timestamp;
		_this->frame_buffers[cur].meta_size = image->meta_size;
		_this->frame_buffers[cur].meta = _this->xmp_buffers[cur];
		if (image->meta_size > 0) {
			memcpy(_this->frame_buffers[cur].meta, image->meta,
					image->meta_size);
		}
	}

	_this->framecount++;
	if ((_this->framecount % _this->image_unit) == 0) {
		mrevent_trigger(&_this->frame_ready);
	}
}

//static void parse_xml(char *xml, _FRAME_T *frame) {
//	frame->xmp_info = true;
//
//	char *q_str = NULL;
//	q_str = strstr(xml, "<quaternion");
//	if (q_str) {
//		sscanf(q_str, "<quaternion x=\"%f\" y=\"%f\" z=\"%f\" w=\"%f\" />", &frame->quaternion.x, &frame->quaternion.y, &frame->quaternion.z, &frame->quaternion.w);
//	}
//	q_str = strstr(xml, "<offset");
//	if (q_str) {
//		sscanf(q_str, "<offset x=\"%f\" y=\"%f\" yaw=\"%f\" horizon_r=\"%f\" />", &frame->offset.x, &frame->offset.y, &frame->offset.z, &frame->offset.w);
//	}
//}

static void* streaming_thread_fnc(void *obj) {
	mjpeg_omx_decoder_private *_this = (mjpeg_omx_decoder_private*) obj;

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
		if (memcmp(images[0]->img_type, "JPEG", 4) != 0) {
			printf("%s : something wrong!\n", __FILE__);
			continue;
		}
		_this->image_unit = num;
		for (int i = 0; i < num; i++) {
			PICAM360_IMAGE_T *image = images[i];
			unsigned char *data = image->pixels[0];
			int data_len = image->stride[0];
			if (data[0] == 0xFF && data[1] == 0xD8) { //SOI
				if (data[data_len - 2] == 0xFF && data[data_len - 1] == 0xD9) { //EOI
					decode(_this, image);
				} else {
					printf("split data\n");
				}
			} else {
				printf("split data\n");
			}
		}
		{
			static int count = 0;
			static float elapsed_sec = 0;
			float _elapsed_sec;
			struct timeval diff;

			struct timeval now;
			gettimeofday(&now, NULL);

			timersub(&now, &images[0]->timestamp, &diff);
			_elapsed_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
			if ((count % 200) == 0) {
				printf("mjpeg decode : %f\n", _elapsed_sec);
			}
			count++;
		}
		for (int i = 0; i < num; i++) {
			PICAM360_IMAGE_T *image = images[i];
			if (images[i]->ref) {
				images[i]->ref->release(images[i]->ref);
			}
		}
	}

	return NULL;
}

static void start(void *user_data) {
	mjpeg_omx_decoder_private *_this = (mjpeg_omx_decoder_private*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}

	_this->run = true;
	pthread_create(&_this->streaming_thread, NULL, streaming_thread_fnc,
			(void*) _this);
}

static void stop(void *user_data) {
	mjpeg_omx_decoder_private *_this = (mjpeg_omx_decoder_private*) user_data;

	if (_this->run) {
		_this->run = false;
		pthread_join(_this->streaming_thread, NULL);
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static int get_image(void *obj, PICAM360_IMAGE_T **images_p, int *num_p,
		int wait_usec) {
	mjpeg_omx_decoder_private *_this = (mjpeg_omx_decoder_private*) obj;

	int res = mrevent_wait(&_this->frame_ready, wait_usec);
	if (res != 0) {
		return -1;
	} else {
		mrevent_reset(&_this->frame_ready);
	}
	int framecount = ((_this->framecount / _this->image_unit) - 1)
			* _this->image_unit;
	for (int i = 0; i < _this->image_unit; i++) {
		int cur = (framecount + i) % BUFFER_NUM;
		images_p[i] = &_this->frame_buffers[cur];
	}
	*num_p = _this->image_unit;

	return 0;
}

static void release(void *obj) {
	mjpeg_omx_decoder_private *_this = (mjpeg_omx_decoder_private*) obj;

	_this->super.stop(&_this->super);

	for (int cur = 0; cur < BUFFER_NUM; cur++) {
		struct jpeg_decompress_struct *cinfop = &_this->cinfos[cur];
		if (cinfop->fd > 0) {
			eglDestroyImageKHR(lg_plugin_host->get_display(),
					(EGLImageKHR) cinfop->client_data);
			glDeleteTextures(1, (GLuint*) &cinfop->fd);
			PFN(jpeg_destroy_decompress)(cinfop);

			cinfop->fd = 0;
		}
	}

	free(obj);
}

static int set_param(void *obj, const char *param, const char *value_str) {
	mjpeg_omx_decoder_private *_this = (mjpeg_omx_decoder_private*) obj;
	return 0;
}

static int get_param(void *obj, const char *param, char *value, int size) {
	mjpeg_omx_decoder_private *_this = (mjpeg_omx_decoder_private*) obj;
	return 0;
}

static void create_decoder(void *user_data, VSTREAMER_T **out_decoder) {
	VSTREAMER_T *decoder = (VSTREAMER_T*) malloc(
			sizeof(_mjpeg_omx_decoder_private));
	memset(decoder, 0, sizeof(_mjpeg_omx_decoder_private));
	strcpy(decoder->name, DECODER_NAME);
	decoder->release = release;
	decoder->start = start;
	decoder->stop = stop;
	decoder->set_param = set_param;
	decoder->get_param = get_param;
	decoder->get_image = get_image;
	decoder->user_data = decoder;

	mjpeg_omx_decoder_private *_private = (mjpeg_omx_decoder_private*) decoder;
	mrevent_init(&_private->frame_ready);

	if (out_decoder) {
		*out_decoder = decoder;
	}
}

static int command_handler(void *user_data, const char *_buff) {
	int opt;
	int ret = 0;
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id,
		uint32_t event_id) {
}

static void init_options(void *user_data, json_t *options) {
}

static void save_options(void *user_data, json_t *options) {
}

void create_mjpeg_decoder_plugin(PLUGIN_HOST_T *plugin_host,
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
		strcpy(factory->name, DECODER_NAME);
		factory->release = release;
		factory->create_vstreamer = create_decoder;
		factory->user_data = factory;

		lg_plugin_host->add_vstreamer_factory(factory);
	}
}
