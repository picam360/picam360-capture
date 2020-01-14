#include "omxjpeg.h"
#include "bcm_host.h"
#include "ilclient.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#include <pthread.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define CHECKED(c, v) if (c) {printf("%s:%d - %s", __FILE__, __LINE__, v); exit(-1);}

#define OMX_INIT_STRUCTURE(a) \
    memset(&(a), 0, sizeof(a)); \
    (a).nSize = sizeof(a); \
    (a).nVersion.nVersion = OMX_VERSION; \
    (a).nVersion.s.nVersionMajor = OMX_VERSION_MAJOR; \
    (a).nVersion.s.nVersionMinor = OMX_VERSION_MINOR; \
    (a).nVersion.s.nRevision = OMX_VERSION_REVISION; \
    (a).nVersion.s.nStep = OMX_VERSION_STEP

typedef struct _omx_jpeg_decompress_private {
	ILCLIENT_T *client;
	COMPONENT_T *video_decode;
	COMPONENT_T *resize;
	COMPONENT_T *egl_render;
	COMPONENT_T *list[4];
	TUNNEL_T tunnel[3];

	uint32_t image_width; //jpeg original image_width
	uint32_t image_height; //jpeg original image_height

	int egl_buffer_num;
	OMX_BUFFERHEADERTYPE *egl_buffer;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	int framecount;
	boolean fill_buffer_done;
	boolean port_settings_done;
} omx_jpeg_decompress_private;

#if 1
#define DECODER_NAME "image_decode"
#define DECODER_INPUT_PORT 320
#define DECODER_OUTPUT_PORT 321
#else
#define DECODER_NAME "video_decode"
#define DECODER_INPUT_PORT 130
#define DECODER_OUTPUT_PORT 131
#endif

static void fill_buffer_done_fnc(void *userdata, COMPONENT_T *comp) {
	j_decompress_ptr cinfo = (j_decompress_ptr) userdata;
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

	//dead lock caution : don't wait ilhost thread in here

	//printf("fill_buffer_done_fnc\n");

	pthread_mutex_lock(&_this->mutex); // this is for avoiding infinity mrevent_wait
	pthread_cond_broadcast(&_this->cond);
	_this->fill_buffer_done = TRUE;
	pthread_mutex_unlock(&_this->mutex);

	_this->framecount++;
}
static void empty_buffer_done_fnc(void *userdata, COMPONENT_T *comp) {
	j_decompress_ptr cinfo = (j_decompress_ptr) userdata;
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

	//dead lock caution : don't wait ilhost thread in here

	//printf("empty_buffer_done_fnc\n");
}
static void port_settings_fnc(void *userdata, struct _COMPONENT_T *comp,
		OMX_U32 data) {
	j_decompress_ptr cinfo = (j_decompress_ptr) userdata;
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

	//dead lock caution : don't wait ilhost thread in here

	//printf("port_settings_fnc\n");
}
static void change_port_settings(j_decompress_ptr cinfo) {
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;
	OMX_ERRORTYPE ret = OMX_ErrorNone;

	OMX_PARAM_PORTDEFINITIONTYPE def;

	// need to setup the input for the resizer with the output of the
	// decoder
	def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	def.nVersion.nVersion = OMX_VERSION;
	def.nPortIndex = DECODER_OUTPUT_PORT;
	ret = OMX_GetParameter(ILC_GET_HANDLE(_this->video_decode),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone, "OMX_GetParameter failed.");

	uint32_t image_width = (unsigned int) def.format.image.nFrameWidth;
	uint32_t image_height = (unsigned int) def.format.image.nFrameHeight;
	uint32_t image_inner_width = MIN(image_width, image_height);

	def.nBufferCountActual = 2;

	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->video_decode),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone, "OMX_SetParameter failed.");

	// tell resizer input what the decoder output will be providing
	def.nPortIndex = 60;

	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->resize),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone, "OMX_SetParameter failed.");

	if (ilclient_setup_tunnel(_this->tunnel, 0, 0) != 0) {
		printf("fail tunnel 0\n");
		return;
	}

	// put resizer in idle state (this allows the outport of the decoder
	// to become enabled)
	ret = ilclient_change_component_state(_this->resize, OMX_StateIdle);
	CHECKED(ret != 0, "ilclient_change_component_state failed.");

	OMX_CONFIG_RECTTYPE omx_crop_req;
	OMX_INIT_STRUCTURE(omx_crop_req);
	omx_crop_req.nPortIndex = 60;
	omx_crop_req.nLeft = (image_width - image_inner_width) / 2;
	omx_crop_req.nWidth = image_inner_width;
	omx_crop_req.nTop = (image_height - image_inner_width) / 2;
	omx_crop_req.nHeight = image_inner_width;
	ret = OMX_SetConfig(ILC_GET_HANDLE(_this->resize),
			OMX_IndexConfigCommonInputCrop, &omx_crop_req);
	CHECKED(ret != OMX_ErrorNone, "OMX_SetConfig failed.");
	//printf("crop %d, %d, %d, %d\n", omx_crop_req.nLeft, omx_crop_req.nTop,
	//		omx_crop_req.nWidth, omx_crop_req.nHeight);

	// query output buffer requirements for resizer
	def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	def.nVersion.nVersion = OMX_VERSION;
	def.nPortIndex = 61;
	ret = OMX_GetParameter(ILC_GET_HANDLE(_this->resize),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone, "OMX_GetParameter failed.");

	def.nBufferCountActual = 2;
	// change output color format and dimensions to match input
	def.format.image.eCompressionFormat = OMX_IMAGE_CodingUnused;
	def.format.image.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
	def.format.image.nFrameWidth = cinfo->image_width;
	def.format.image.nFrameHeight = cinfo->image_height;
	def.format.image.nStride = 0;
	//def.format.image.nSliceHeight = 16;
	def.format.image.nSliceHeight = 0;
	def.format.image.bFlagErrorConcealment = OMX_FALSE;

	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->resize),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone, "OMX_SetParameter failed.");

	// grab output requirements again to get actual buffer size
	// requirement (and buffer count requirement!)
	ret = OMX_GetParameter(ILC_GET_HANDLE(_this->resize),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone, "OMX_GetParameter failed.");

	// move resizer into executing state
	ret = ilclient_change_component_state(_this->resize, OMX_StateExecuting);
	CHECKED(ret != 0, "ilclient_change_component_state failed.");

	// show some logging so user knows it's working
//	printf("Width: %u Height: %u Output Color Format: 0x%x Buffer Size: %u\n",
//			(unsigned int) def.format.image.nFrameWidth,
//			(unsigned int) def.format.image.nFrameHeight,
//			(unsigned int) def.format.image.eColorFormat,
//			(unsigned int) def.nBufferSize);
//	fflush(stdout);

	//resize -> egl_render

	//setup tunnel
	if (ilclient_setup_tunnel(_this->tunnel + 1, 0, 0) != 0) {
		printf("fail tunnel 1\n");
		return;
	}

	// Set lg_egl_render to idle
	ret = ilclient_change_component_state(_this->egl_render, OMX_StateIdle);
	CHECKED(ret != 0, "ilclient_change_component_state failed.");

	// Obtain the information about the output port.
	OMX_PARAM_PORTDEFINITIONTYPE port_format;
	OMX_INIT_STRUCTURE(port_format);
	port_format.nPortIndex = 221;
	ret = OMX_GetParameter(ILC_GET_HANDLE(_this->egl_render),
			OMX_IndexParamPortDefinition, &port_format);
	CHECKED(ret != OMX_ErrorNone, "OMX_GetParameter failed.");

	EGLImageKHR egl_image = (EGLImageKHR) cinfo->client_data;

	port_format.nBufferCountActual = 1;
	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->egl_render),
			OMX_IndexParamPortDefinition, &port_format);
	CHECKED(ret != OMX_ErrorNone, "OMX_SetParameter failed.");

	// Enable the output port and tell lg_egl_render to use the texture as a buffer
	//ilclient_enable_port(lg_egl_render, 221); THIS BLOCKS SO CAN'T BE USED
	ret = OMX_SendCommand(ILC_GET_HANDLE(_this->egl_render),
			OMX_CommandPortEnable, 221, NULL);
	CHECKED(ret != OMX_ErrorNone, "OMX_SendCommand failed.");

	//for (int i = 0; i < _this->egl_buffer_num; i++)
	{
		OMX_STATETYPE state;
		OMX_GetState(ILC_GET_HANDLE(_this->egl_render), &state);
		if (state != OMX_StateIdle) {
			if (state != OMX_StateLoaded) {
				ilclient_change_component_state(_this->egl_render,
						OMX_StateLoaded);
			}
			ret = ilclient_change_component_state(_this->egl_render,
					OMX_StateIdle);
			CHECKED(ret != 0, "ilclient_change_component_state failed.");
		}
		ret = OMX_UseEGLImage(ILC_GET_HANDLE(_this->egl_render),
				&_this->egl_buffer, 221, (void*) 0, egl_image);
		CHECKED(ret != OMX_ErrorNone, "OMX_UseEGLImage failed.");
	}

	// Set lg_egl_render to executing
	ilclient_change_component_state(_this->egl_render, OMX_StateExecuting);
	CHECKED(ret != 0, "ilclient_change_component_state failed.");
}

OMXJPEG_FN_DEFINE(void, jpeg_CreateDecompress,
		(j_decompress_ptr cinfo, int version, size_t structsize)) {
	cinfo->src = (struct jpeg_source_mgr*) malloc(
			sizeof(struct jpeg_source_mgr));
	memset(cinfo->src, 0, sizeof(struct jpeg_source_mgr));

	cinfo->comp_info = (jpeg_component_info*) malloc(
			sizeof(jpeg_component_info));
	memset(cinfo->comp_info, 0, sizeof(jpeg_component_info));

	omx_jpeg_decompress_private *_this = (omx_jpeg_decompress_private*) malloc(
			sizeof(omx_jpeg_decompress_private));
	memset(_this, 0, sizeof(omx_jpeg_decompress_private));
	cinfo->master = (struct jpeg_decomp_master*) _this;

	pthread_mutex_init(&_this->mutex, 0);
	pthread_cond_init(&_this->cond, 0);

	int ret = 0;
	unsigned int data_len = 0;

	_this->client = ilclient_init();
	CHECKED(_this->client == NULL, "ilclient_init failed.");

	ret = OMX_Init();
	CHECKED(ret != OMX_ErrorNone, "OMX_Init failed.");

	// callback
	ilclient_set_fill_buffer_done_callback(_this->client, fill_buffer_done_fnc,
			(void*) cinfo);
	ilclient_set_empty_buffer_done_callback(_this->client,
			empty_buffer_done_fnc, (void*) cinfo);
	ilclient_set_port_settings_callback(_this->client, port_settings_fnc,
			(void*) cinfo);

	// create video_decode
	ret = ilclient_create_component(_this->client, &_this->video_decode,
			(char*) DECODER_NAME,
			(ILCLIENT_CREATE_FLAGS_T)(
					ILCLIENT_DISABLE_ALL_PORTS
							| ILCLIENT_ENABLE_INPUT_BUFFERS));
	CHECKED(ret != 0, "ilclient_create_component failed\n");

	_this->list[0] = _this->video_decode;

	// create resize
	ret = ilclient_create_component(_this->client, &_this->resize,
			(char*) "resize",
			(ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS));
	CHECKED(ret != 0, "ilclient_create_component failed\n");

	_this->list[1] = _this->resize;

	// create lg_egl_render
	ret = ilclient_create_component(_this->client, &_this->egl_render,
			(char*) "egl_render",
			(ILCLIENT_CREATE_FLAGS_T)(
					ILCLIENT_DISABLE_ALL_PORTS
							| ILCLIENT_ENABLE_OUTPUT_BUFFERS));
	CHECKED(ret != 0, "ilclient_create_component failed\n");

	_this->list[2] = _this->egl_render;

	set_tunnel(_this->tunnel, _this->video_decode, DECODER_OUTPUT_PORT,
			_this->resize, 60);
	set_tunnel(_this->tunnel + 1, _this->resize, 61, _this->egl_render, 220);

	ret = ilclient_change_component_state(_this->video_decode, OMX_StateIdle);
	CHECKED(ret != 0, "ilclient_change_component_state failed\n");

	OMX_PARAM_PORTDEFINITIONTYPE def = { 0 };
	def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	def.nVersion.nVersion = OMX_VERSION;
	def.nPortIndex = DECODER_INPUT_PORT;
	ret = OMX_GetParameter(ILC_GET_HANDLE(_this->video_decode),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_GetParameter failed for encode port out.");

	def.format.image.eCompressionFormat = OMX_IMAGE_CodingJPEG;
	def.format.image.eColorFormat = OMX_COLOR_FormatUnused;

	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->video_decode),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_SetParameter failed for input format definition.");

	ret = ilclient_enable_port_buffers(_this->video_decode, DECODER_INPUT_PORT,
			NULL, NULL, NULL);
	CHECKED(ret != 0, "ilclient_enable_port_buffers failed\n");

	// Set video_decode to executing
	ilclient_change_component_state(_this->video_decode, OMX_StateExecuting);
}
OMXJPEG_FN_DEFINE(void, jpeg_destroy_decompress, (j_decompress_ptr cinfo)) {
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

	// callback
	ilclient_set_fill_buffer_done_callback(_this->client, NULL, (void*) cinfo);
	ilclient_set_empty_buffer_done_callback(_this->client, NULL, (void*) cinfo);
	ilclient_set_port_settings_callback(_this->client, NULL, (void*) cinfo);

	int ret = 0;
	OMX_BUFFERHEADERTYPE *buf = NULL;

	buf = ilclient_get_input_buffer(_this->video_decode, DECODER_INPUT_PORT, 1);
	buf->nFilledLen = 0;
	buf->nFlags = OMX_BUFFERFLAG_TIME_UNKNOWN | OMX_BUFFERFLAG_EOS;

	ret = OMX_EmptyThisBuffer(ILC_GET_HANDLE(_this->video_decode), buf);
	CHECKED(ret != OMX_ErrorNone, "OMX_EmptyThisBuffer failed\n");

	// need to flush the renderer to allow video_decode to disable its input port
	ilclient_flush_tunnels(_this->tunnel, 0);

	ilclient_disable_port_buffers(_this->video_decode, DECODER_INPUT_PORT, NULL,
			NULL, NULL);

	ilclient_disable_tunnel(_this->tunnel);
	ilclient_disable_tunnel(_this->tunnel + 1);
	ilclient_teardown_tunnels(_this->tunnel);

	ilclient_state_transition(_this->list, OMX_StateIdle);

	ilclient_cleanup_components(_this->list);

	OMX_Deinit();

	ilclient_destroy(_this->client);

	free(cinfo->comp_info);
	free(cinfo->src);
	free(cinfo->master);
}
OMXJPEG_FN_DEFINE(void, jpeg_mem_src,
		(j_decompress_ptr cinfo, unsigned char * inbuffer, unsigned long insize)) {
	cinfo->src->next_input_byte = inbuffer;
	cinfo->src->bytes_in_buffer = insize;
}
OMXJPEG_FN_DEFINE(int, jpeg_read_header,
		(j_decompress_ptr cinfo, boolean require_image)) {
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

	int image_width = 0;
	int image_height = 0;

	uint8_t *data = (uint8_t*) cinfo->src->next_input_byte;
	int data_len = cinfo->src->bytes_in_buffer;
	for (int i = 0; i < data_len; i++) {
		if (data[i] == 0xFF && data[i + 1] == 0xC0) {
			image_height += data[i + 5] << 8;
			image_height += data[i + 6] << 0;

			image_width += data[i + 7] << 8;
			image_width += data[i + 8] << 0;
			break;
		}
	}

	uint32_t image_inner_width = MIN(image_width, image_height);

	uint32_t image_width_2n;
	for (image_width_2n = 64; image_width_2n < 2048; image_width_2n *= 2) {
		if (image_width_2n >= image_inner_width) {
			break;
		}
	}
	cinfo->image_width = image_width_2n;
	cinfo->image_height = image_width_2n;

	_this->image_width = image_width;
	_this->image_height = image_height;

	return 0;
}

OMXJPEG_FN_DEFINE(boolean, jpeg_start_decompress, (j_decompress_ptr cinfo)) {
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

	return TRUE;
}
OMXJPEG_FN_DEFINE(JDIMENSION, jpeg_read_raw_data,
		(j_decompress_ptr cinfo, JSAMPIMAGE data, JDIMENSION max_lines)) {
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

}
OMXJPEG_FN_DEFINE(boolean, jpeg_finish_decompress, (j_decompress_ptr cinfo)) {
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

	_this->fill_buffer_done = FALSE;

	uint8_t *data = (uint8_t*) cinfo->src->next_input_byte;
	int data_len = cinfo->src->bytes_in_buffer;

	if (!data_len) {
		return FALSE;
	}

	int ret = 0;
	OMX_BUFFERHEADERTYPE *buf = NULL;

	for (int cur = 0; cur < data_len; cur += buf->nAllocLen) {
		buf = ilclient_get_input_buffer(_this->video_decode, DECODER_INPUT_PORT,
				1);

		buf->nFilledLen = MIN(data_len - cur, buf->nAllocLen);
		memcpy(buf->pBuffer, data + cur, buf->nFilledLen);

		buf->nOffset = 0;
		buf->nFlags = 0;
		if (cur + buf->nFilledLen == data_len) {
			buf->nFlags |= OMX_BUFFERFLAG_EOS;
			//printf("finish\n");
		}

		ret = OMX_EmptyThisBuffer(ILC_GET_HANDLE(_this->video_decode), buf);
		CHECKED(ret != OMX_ErrorNone, "OMX_FillThisBuffer failed\n");
	}
	if (!_this->port_settings_done) {
		while (1) {
			ret = ilclient_wait_for_event(_this->video_decode,
					OMX_EventPortSettingsChanged, DECODER_OUTPUT_PORT, 0, 0, 1,
					0, 5);
			//ret = ilclient_remove_event(_this->video_decode,
			//		OMX_EventPortSettingsChanged, DECODER_OUTPUT_PORT, 0, 0, 1);
			if (ret == 0) {
				//printf("change port_settings\n");
				_this->port_settings_done = TRUE;
				change_port_settings(cinfo);
				break;
			}
		}
	}

	_this->fill_buffer_done = FALSE;
	ret = OMX_FillThisBuffer(ilclient_get_handle(_this->egl_render),
			_this->egl_buffer);
	CHECKED(ret != OMX_ErrorNone, "OMX_FillThisBuffer failed\n");

	pthread_mutex_lock(&_this->mutex);
	if (_this->fill_buffer_done != TRUE) {
		pthread_cond_wait(&_this->cond, &_this->mutex);
	}
	pthread_mutex_unlock(&_this->mutex);

	return TRUE;
}
