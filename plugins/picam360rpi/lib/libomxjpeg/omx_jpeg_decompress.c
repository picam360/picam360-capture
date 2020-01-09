#include "omxjpeg.h"
#include "bcm_host.h"
#include "ilclient.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#include <pthread.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

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

	int texture_width;
	int texture_height;
	OMX_BUFFERHEADERTYPE *egl_buffer;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	boolean fill_buffer_done;
} omx_jpeg_decompress_private;

static void my_fill_buffer_done(void *data, COMPONENT_T *comp) {
	omx_jpeg_decompress_private *_this = (omx_jpeg_decompress_private*) data;

	pthread_mutex_lock(&_this->mutex); // this is for avoiding infinity mrevent_wait
	pthread_cond_broadcast(&_this->cond);
	_this->fill_buffer_done = TRUE;
	pthread_mutex_unlock(&_this->mutex);
}

OMXJPEG_FN_DEFINE(void, jpeg_CreateDecompress,
		(j_decompress_ptr cinfo, int version, size_t structsize)) {
	cinfo->src = (struct jpeg_source_mgr*) malloc(
			sizeof(struct jpeg_source_mgr));
	memset(cinfo->src, 0, sizeof(struct jpeg_source_mgr));

	omx_jpeg_decompress_private *_this = (omx_jpeg_decompress_private*) malloc(
			sizeof(omx_jpeg_decompress_private));
	memset(_this, 0, sizeof(omx_jpeg_decompress_private));
	cinfo->master = (struct jpeg_decomp_master*) _this;

	pthread_mutex_init(&_this->mutex, 0);
	pthread_cond_init(&_this->cond, 0);

	OMX_VIDEO_PARAM_PORTFORMATTYPE format;
	int status = 0;
	unsigned int data_len = 0;

	if ((_this->client = ilclient_init()) == NULL) {
		return;
	}

	if (OMX_Init() != OMX_ErrorNone) {
		ilclient_destroy(_this->client);
		return;
	}

	// callback
	ilclient_set_fill_buffer_done_callback(_this->client, my_fill_buffer_done,
			(void*) _this);

	// create video_decode
	if (ilclient_create_component(_this->client, &_this->video_decode,
			(char*) "video_decode",
			(ILCLIENT_CREATE_FLAGS_T)(
					ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_INPUT_BUFFERS))
			!= 0)
		status = -14;
	_this->list[0] = _this->video_decode;

	// create resize
	if (status == 0
			&& ilclient_create_component(_this->client, &_this->resize,
					(char*) "resize",
					(ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS)) != 0)
		status = -14;
	_this->list[1] = _this->resize;

	// create lg_egl_render
	if (status == 0
			&& ilclient_create_component(_this->client, &_this->egl_render,
					(char*) "egl_render",
					(ILCLIENT_CREATE_FLAGS_T)(
							ILCLIENT_DISABLE_ALL_PORTS
									| ILCLIENT_ENABLE_OUTPUT_BUFFERS)) != 0)
		status = -14;
	_this->list[2] = _this->egl_render;

	set_tunnel(_this->tunnel, _this->video_decode, 131, _this->resize, 60);
	set_tunnel(_this->tunnel + 1, _this->resize, 61, _this->egl_render, 220);

	if (status == 0)
		ilclient_change_component_state(_this->video_decode, OMX_StateIdle);

	memset(&format, 0, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
	format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
	format.nVersion.nVersion = OMX_VERSION;
	format.nPortIndex = 130;
	format.eCompressionFormat = OMX_VIDEO_CodingMJPEG;
	if (status == 0
			&& OMX_SetParameter(ILC_GET_HANDLE(_this->video_decode),
					OMX_IndexParamVideoPortFormat, &format) != OMX_ErrorNone) {
		status = -14;
	}
	if (status == 0
			&& ilclient_enable_port_buffers(_this->video_decode, 130, NULL,
					NULL, NULL) != 0) {
		status = -14;
	}

	ilclient_change_component_state(_this->video_decode, OMX_StateExecuting);
}
OMXJPEG_FN_DEFINE(void, jpeg_destroy_decompress, (j_decompress_ptr cinfo)) {
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

	int status = 0;
	OMX_BUFFERHEADERTYPE *buf = NULL;

	buf = ilclient_get_input_buffer(_this->video_decode, 130, 1);
	buf->nFilledLen = 0;
	buf->nFlags = OMX_BUFFERFLAG_TIME_UNKNOWN | OMX_BUFFERFLAG_EOS;

	if (OMX_EmptyThisBuffer(ILC_GET_HANDLE(_this->video_decode), buf)
			!= OMX_ErrorNone)
		status = -20;

	// need to flush the renderer to allow video_decode to disable its input port
	ilclient_flush_tunnels(_this->tunnel, 0);

	ilclient_disable_port_buffers(_this->video_decode, 130, NULL, NULL, NULL);

	ilclient_disable_tunnel(_this->tunnel);
	ilclient_teardown_tunnels(_this->tunnel);

	ilclient_state_transition(_this->list, OMX_StateIdle);
	ilclient_state_transition(_this->list, OMX_StateLoaded);

	ilclient_cleanup_components(_this->list);

	OMX_Deinit();

	ilclient_destroy(_this->client);

	if (cinfo->comp_info) {
		free(cinfo->comp_info);
	}
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

	if (_this->egl_buffer) {
		//already done
		return 0;
	}

	int image_width = 0;
	int image_height = 0;
	int image_inner_width = 0;

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
	cinfo->image_width = image_width;
	cinfo->image_height = image_height;
	image_inner_width = MIN(image_width, image_height);
	_this->texture_width = image_inner_width;
	_this->texture_height = image_inner_width;

	OMX_ERRORTYPE omx_err = OMX_ErrorNone;

	OMX_PARAM_PORTDEFINITIONTYPE portdef;

	portdef.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	portdef.nVersion.nVersion = OMX_VERSION;
	portdef.nPortIndex = 60;
	OMX_GetParameter(ILC_GET_HANDLE(_this->resize),
			OMX_IndexParamPortDefinition, &portdef);

	portdef.format.image.nFrameWidth = image_width;
	portdef.format.image.nFrameHeight = image_height;

	OMX_SetParameter(ILC_GET_HANDLE(_this->resize),
			OMX_IndexParamPortDefinition, &portdef);

	if (ilclient_setup_tunnel(_this->tunnel, 0, 0) != 0) {
		printf("fail tunnel 0\n");
		return -7;
	}

	// put resizer in idle state (this allows the outport of the decoder
	// to become enabled)
	ilclient_change_component_state(_this->resize, OMX_StateIdle);

	OMX_CONFIG_RECTTYPE omx_crop_req;
	OMX_INIT_STRUCTURE(omx_crop_req);
	omx_crop_req.nPortIndex = 60;
	omx_crop_req.nLeft = (image_width - image_inner_width) / 2;
	omx_crop_req.nWidth = image_inner_width;
	omx_crop_req.nTop = (image_height - image_inner_width) / 2;
	omx_crop_req.nHeight = image_inner_width;
	OMX_SetConfig(ILC_GET_HANDLE(_this->resize), OMX_IndexConfigCommonInputCrop,
			&omx_crop_req);
	//printf("crop %d, %d, %d, %d\n", omx_crop_req.nLeft, omx_crop_req.nTop,
	//		omx_crop_req.nWidth, omx_crop_req.nHeight);

	// query output buffer requirements for resizer
	portdef.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	portdef.nVersion.nVersion = OMX_VERSION;
	portdef.nPortIndex = 61;
	OMX_GetParameter(ILC_GET_HANDLE(_this->resize),
			OMX_IndexParamPortDefinition, &portdef);

	// change output color format and dimensions to match input
	portdef.format.image.eCompressionFormat = OMX_IMAGE_CodingUnused;
	portdef.format.image.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
	portdef.format.image.nFrameWidth = _this->texture_width;
	portdef.format.image.nFrameHeight = _this->texture_height;
	portdef.format.image.nStride = 0;
	portdef.format.image.nSliceHeight = 16;
	portdef.format.image.bFlagErrorConcealment = OMX_FALSE;

	OMX_SetParameter(ILC_GET_HANDLE(_this->resize),
			OMX_IndexParamPortDefinition, &portdef);

	// grab output requirements again to get actual buffer size
	// requirement (and buffer count requirement!)
	OMX_GetParameter(ILC_GET_HANDLE(_this->resize),
			OMX_IndexParamPortDefinition, &portdef);

	// move resizer into executing state
	ilclient_change_component_state(_this->resize, OMX_StateExecuting);

	// show some logging so user knows it's working
	printf("Width: %u Height: %u Output Color Format: 0x%x Buffer Size: %u\n",
			(unsigned int) portdef.format.image.nFrameWidth,
			(unsigned int) portdef.format.image.nFrameHeight,
			(unsigned int) portdef.format.image.eColorFormat,
			(unsigned int) portdef.nBufferSize);
	fflush(stdout);

	//resize -> egl_render

	//setup tunnel
	if (ilclient_setup_tunnel(_this->tunnel + 1, 0, 0) != 0) {
		printf("fail tunnel 1\n");
		return -7;
	}

	// Set lg_egl_render to idle
	ilclient_change_component_state(_this->egl_render, OMX_StateIdle);

	// Obtain the information about the output port.
	OMX_PARAM_PORTDEFINITIONTYPE port_format;
	OMX_INIT_STRUCTURE(port_format);
	port_format.nPortIndex = 221;
	omx_err = OMX_GetParameter(ILC_GET_HANDLE(_this->egl_render),
			OMX_IndexParamPortDefinition, &port_format);
	if (omx_err != OMX_ErrorNone) {
		printf(
				"%s - OMX_GetParameter OMX_IndexParamPortDefinition omx_err(0x%08x)",
				__func__, omx_err);
		exit(1);
	}

	port_format.nBufferCountActual = 1;
	omx_err = OMX_SetParameter(ILC_GET_HANDLE(_this->egl_render),
			OMX_IndexParamPortDefinition, &port_format);
	if (omx_err != OMX_ErrorNone) {
		printf(
				"%s - OMX_SetParameter OMX_IndexParamPortDefinition omx_err(0x%08x)",
				__func__, omx_err);
		exit(1);
	}

	// Enable the output port and tell lg_egl_render to use the texture as a buffer
	//ilclient_enable_port(lg_egl_render, 221); THIS BLOCKS SO CAN'T BE USED
	if (OMX_SendCommand(ILC_GET_HANDLE(_this->egl_render),
			OMX_CommandPortEnable, 221, NULL) != OMX_ErrorNone) {
		printf("OMX_CommandPortEnable failed.\n");
		exit(1);
	}

	{
		OMX_STATETYPE state;
		OMX_GetState(ILC_GET_HANDLE(_this->egl_render), &state);
		if (state != OMX_StateIdle) {
			if (state != OMX_StateLoaded) {
				ilclient_change_component_state(_this->egl_render,
						OMX_StateLoaded);
			}
			ilclient_change_component_state(_this->egl_render, OMX_StateIdle);
		}
		omx_err = OMX_UseEGLImage(ILC_GET_HANDLE(_this->egl_render),
				&_this->egl_buffer, 221, (void*) 0,
				(EGLImageKHR) cinfo->client_data);
		if (omx_err != OMX_ErrorNone) {
			printf("OMX_UseEGLImage failed. 0x%x\n", omx_err);
			exit(1);
		}
	}

	// Set lg_egl_render to executing
	ilclient_change_component_state(_this->egl_render, OMX_StateExecuting);

	return 0;
}

OMXJPEG_FN_DEFINE(boolean, jpeg_start_decompress, (j_decompress_ptr cinfo)) {
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

	_this->fill_buffer_done = FALSE;

	cinfo->comp_info = (jpeg_component_info*) malloc(
			sizeof(jpeg_component_info));
	memset(cinfo->comp_info, 0, sizeof(jpeg_component_info));

	uint8_t *data = (uint8_t*) cinfo->src->next_input_byte;
	int data_len = cinfo->src->bytes_in_buffer;

	int status = 0;
	OMX_BUFFERHEADERTYPE *buf = NULL;

	buf = ilclient_get_input_buffer(_this->video_decode, 130, 1);

	data_len = MIN((int )buf->nAllocLen, data_len);
	memcpy(buf->pBuffer, data, data_len);

	if (!data_len) {
		return FALSE;
	}
	buf->nFilledLen = data_len;

	buf->nOffset = 0;
	buf->nFlags = OMX_BUFFERFLAG_TIME_UNKNOWN;
	buf->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;

	if (OMX_EmptyThisBuffer(ILC_GET_HANDLE(_this->video_decode), buf)
			!= OMX_ErrorNone) {
		status = -6;
		return FALSE;
	}

	return TRUE;
}
OMXJPEG_FN_DEFINE(JDIMENSION, jpeg_read_raw_data,
		(j_decompress_ptr cinfo, JSAMPIMAGE data, JDIMENSION max_lines)) {
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;

	if (OMX_FillThisBuffer(ilclient_get_handle(_this->egl_render),
			_this->egl_buffer) != OMX_ErrorNone) {
		printf("OMX_FillThisBuffer failed in callback\n");
		//exit(1);
	}
	pthread_mutex_lock(&_this->mutex);
	if (_this->fill_buffer_done != TRUE) {
		pthread_cond_wait(&_this->cond, &_this->mutex);
	}
	pthread_mutex_unlock(&_this->mutex);

}
OMXJPEG_FN_DEFINE(boolean, jpeg_finish_decompress, (j_decompress_ptr cinfo)) {
	omx_jpeg_decompress_private *_this =
			(omx_jpeg_decompress_private*) cinfo->master;
	return TRUE;
}
