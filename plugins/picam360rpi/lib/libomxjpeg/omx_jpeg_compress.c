#include "omxjpeg.h"
#include "bcm_host.h"
#include "ilclient.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#include <pthread.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define CHECKED(c, v) if (c) {printf(v); exit(-1);}

#define IMAGE_ENCODE

#define OMX_INIT_STRUCTURE(a) \
    memset(&(a), 0, sizeof(a)); \
    (a).nSize = sizeof(a); \
    (a).nVersion.nVersion = OMX_VERSION; \
    (a).nVersion.s.nVersionMajor = OMX_VERSION_MAJOR; \
    (a).nVersion.s.nVersionMinor = OMX_VERSION_MINOR; \
    (a).nVersion.s.nRevision = OMX_VERSION_REVISION; \
    (a).nVersion.s.nStep = OMX_VERSION_STEP

typedef struct _omx_jpeg_compress_private {
	ILCLIENT_T *client;
	COMPONENT_T *image_encode;

	OMX_BUFFERHEADERTYPE *in_buffer;
	OMX_BUFFERHEADERTYPE *out_buffer;
	unsigned char **outbuffer;
	unsigned long *outsize;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	int framecount;
	boolean fill_buffer_done;
	boolean port_settings_done;
} omx_jpeg_compress_private;

static void fill_buffer_done_fnc(void *userdata, COMPONENT_T *comp) {
	//printf("fill_buffer_done_fnc\n");

	j_compress_ptr cinfo = (j_compress_ptr) userdata;
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;

	pthread_mutex_lock(&_this->mutex); // this is for avoiding infinity mrevent_wait
	pthread_cond_broadcast(&_this->cond);
	_this->fill_buffer_done = TRUE;
	pthread_mutex_unlock(&_this->mutex);

	_this->framecount++;
}
static void empty_buffer_done_fnc(void *userdata, COMPONENT_T *comp) {
	j_compress_ptr cinfo = (j_compress_ptr) userdata;
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;
	//printf("empty_buffer_done_fnc\n");
}
static void port_settings_fnc(void *userdata, struct _COMPONENT_T *comp,
		OMX_U32 data) {
	j_compress_ptr cinfo = (j_compress_ptr) userdata;
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;
	//printf("port_settings_fnc\n");
	//dead lock caution : don't wait ilhost thread in here
}

OMXJPEG_FN_DEFINE(void, jpeg_CreateCompress,
		(j_compress_ptr cinfo, int version, size_t structsize)) {
	cinfo->dest = (struct jpeg_destination_mgr*) malloc(
			sizeof(struct jpeg_destination_mgr));
	memset(cinfo->dest, 0, sizeof(struct jpeg_destination_mgr));

	omx_jpeg_compress_private *_this = (omx_jpeg_compress_private*) malloc(
			sizeof(omx_jpeg_compress_private));
	memset(_this, 0, sizeof(omx_jpeg_compress_private));
	cinfo->master = (struct jpeg_comp_master*) _this;

	pthread_mutex_init(&_this->mutex, 0);
	pthread_cond_init(&_this->cond, 0);

	int ret = 0;
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
	ilclient_set_fill_buffer_done_callback(_this->client, fill_buffer_done_fnc,
			(void*) cinfo);
	ilclient_set_empty_buffer_done_callback(_this->client,
			empty_buffer_done_fnc, (void*) cinfo);
	ilclient_set_port_settings_callback(_this->client, port_settings_fnc,
			(void*) cinfo);

#ifdef IMAGE_ENCODE
	// create image_encode
	ret = ilclient_create_component(_this->client, &_this->image_encode,
			(char*) "image_encode",
			(ILCLIENT_CREATE_FLAGS_T)(
					ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_INPUT_BUFFERS
							| ILCLIENT_ENABLE_OUTPUT_BUFFERS));
	CHECKED(ret != 0, "ILCient image_encode component creation failed.");
#else
	ret = ilclient_create_component(_this->client, &_this->image_encode,
			(char*) "video_encode",
			(ILCLIENT_CREATE_FLAGS_T)(
					ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_INPUT_BUFFERS
							| ILCLIENT_ENABLE_OUTPUT_BUFFERS));
	CHECKED(ret != 0, "ILCient video_encode component creation failed.");

#endif

}
OMXJPEG_FN_DEFINE(void, jpeg_destroy_compress, (j_compress_ptr cinfo)) {
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;

	//Teardown similar to hello_encode
	ilclient_change_component_state(_this->image_encode, OMX_StateIdle);
	ilclient_disable_port_buffers(_this->image_encode, 340, NULL, NULL, NULL);
	ilclient_disable_port_buffers(_this->image_encode, 341, NULL, NULL, NULL);

	//ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
	ilclient_change_component_state(_this->image_encode, OMX_StateLoaded);

	COMPONENT_T *list[] = { _this->image_encode, NULL };
	ilclient_cleanup_components(list);
	ilclient_destroy(_this->client);

}
OMXJPEG_FN_DEFINE(void, jpeg_suppress_tables,
		(j_compress_ptr cinfo, boolean suppress)) {
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;

}
OMXJPEG_FN_DEFINE(void, jpeg_mem_dest,
		(j_compress_ptr cinfo, unsigned char ** outbuffer,unsigned long * outsize)) {
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;

	_this->outbuffer = outbuffer;
	_this->outsize = outsize;

	cinfo->dest->next_output_byte = *outbuffer;
	cinfo->dest->free_in_buffer = *outsize;
}
OMXJPEG_FN_DEFINE(void, jpeg_set_defaults, (j_compress_ptr cinfo)) {
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;

}
OMXJPEG_FN_DEFINE(void, jpeg_set_quality,
		(j_compress_ptr cinfo, int quality, boolean force_baseline)) {
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;
#ifdef IMAGE_ENCODE
	int ret = 0;

	//Set the encoder quality
	OMX_IMAGE_PARAM_QFACTORTYPE qfactor = { 0 };
	qfactor.nSize = sizeof(OMX_IMAGE_PARAM_QFACTORTYPE);
	qfactor.nVersion.nVersion = OMX_VERSION;
	qfactor.nPortIndex = 341;
	qfactor.nQFactor = quality;

	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->image_encode),
			OMX_IndexParamQFactor, &qfactor);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_SetParameter failed for setting encoder quality.");
#endif
}
OMXJPEG_FN_DEFINE(void, jpeg_set_hardware_acceleration_parameters_enc,
		(j_compress_ptr cinfo, boolean hw_acceleration, unsigned int defaultBuffSize, unsigned int defaultWidth, unsigned int defaultHeight )) {
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;

}
OMXJPEG_FN_DEFINE(JDIMENSION, jpeg_write_raw_data,
		(j_compress_ptr cinfo, JSAMPIMAGE data, JDIMENSION num_lines)) {
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;

}
OMXJPEG_FN_DEFINE(void, jpeg_start_compress,
		(j_compress_ptr cinfo, boolean write_all_tables)) {
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;

	if (_this->port_settings_done) {
		return;
	}
	_this->port_settings_done = TRUE;

	int ret = 0;
	OMX_ERRORTYPE omx_err = OMX_ErrorNone;

#ifdef IMAGE_ENCODE
	//Set input definition to the encoder
	OMX_PARAM_PORTDEFINITIONTYPE def = { 0 };
	def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	def.nVersion.nVersion = OMX_VERSION;
	def.nPortIndex = 340;
	ret = OMX_GetParameter(ILC_GET_HANDLE(_this->image_encode),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone, "OMX_GetParameter failed for encode port in.");

	//We allocate 1 input buffers.
	def.nBufferCountActual = 1;
	def.format.image.nFrameWidth = cinfo->image_width;
	def.format.image.nFrameHeight = cinfo->image_height;
	//16 byte alignment. I don't know if these also hold for image encoding.
	def.format.image.nSliceHeight = (cinfo->image_height + 15) & ~15;
	//Must be manually defined to ensure sufficient size if stride needs to be rounded up to multiple of 32.
	def.format.image.nStride = ((cinfo->image_height + 31) & ~31) * 4;
	def.nBufferSize = def.format.image.nStride * def.format.image.nSliceHeight;
	//def.nBufferSize = sizeof(OMX_BRCMVEGLIMAGETYPE);
	def.format.image.bFlagErrorConcealment = OMX_FALSE;
	def.format.image.eColorFormat = OMX_COLOR_Format32bitABGR8888; // OMX_COLOR_FormatBRCMEGL; //OMX_COLOR_Format24bitBGR888; //OMX_COLOR_Format32bitABGR8888;//OMX_COLOR_FormatYUV420PackedPlanar;

	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->image_encode),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_SetParameter failed for input format definition.");

//	//Set the output format of the encoder
//	OMX_IMAGE_PARAM_PORTFORMATTYPE format = { 0 };
//	format.nSize = sizeof(OMX_IMAGE_PARAM_PORTFORMATTYPE);
//	format.nVersion.nVersion = OMX_VERSION;
//	format.nPortIndex = 341;
//	format.eCompressionFormat = OMX_IMAGE_CodingJPEG;
//
//	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->image_encode),
//			OMX_IndexParamImagePortFormat, &format);
//	CHECKED(ret != OMX_ErrorNone,
//			"OMX_SetParameter failed for setting encoder output format.");

	def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	def.nVersion.nVersion = OMX_VERSION;
	def.nPortIndex = 341;
	ret = OMX_GetParameter(ILC_GET_HANDLE(_this->image_encode),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_GetParameter failed for encode port out.");

	def.nBufferSize = cinfo->image_width * cinfo->image_height;
	def.format.image.eCompressionFormat = OMX_IMAGE_CodingJPEG;
	def.format.image.eColorFormat = OMX_COLOR_FormatUnused;

	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->image_encode),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_SetParameter failed for input format definition.");

	ret = ilclient_change_component_state(_this->image_encode, OMX_StateIdle);
	CHECKED(ret != 0, "ILClient failed to change encoder to idle state.");
	ret = ilclient_enable_port_buffers(_this->image_encode, 340, NULL, NULL,
			NULL);
	CHECKED(ret != 0, "ILClient failed to enable input buffers.");
	ret = ilclient_enable_port_buffers(_this->image_encode, 341, NULL, NULL,
			NULL);
	CHECKED(ret != 0, "ILClient failed to enable output buffers.");
	ret = ilclient_change_component_state(_this->image_encode,
			OMX_StateExecuting);
	CHECKED(ret != 0, "ILClient failed to change encoder to executing stage.");
#else
	//Set input definition to the encoder
	OMX_PARAM_PORTDEFINITIONTYPE def = { };
	def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	def.nVersion.nVersion = OMX_VERSION;
	def.nPortIndex = 200;
	ret = OMX_GetParameter(ILC_GET_HANDLE(_this->image_encode),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone, "OMX_GetParameter failed for encode port in.");

	def.format.video.nFrameWidth = cinfo->image_width;
	def.format.video.nFrameHeight = cinfo->image_height;
	def.format.video.xFramerate = 5 << 16;

	//Must be a multiple of 16
	def.format.video.nSliceHeight = (cinfo->image_height + 15) & ~15;
	//Must be a multiple of 32
	def.format.video.nStride = ((cinfo->image_height + 31) & ~31) * 4;
	def.format.video.eColorFormat = OMX_COLOR_Format24bitBGR888; //OMX_COLOR_Format32bitABGR8888;//OMX_COLOR_FormatYUV420PackedPlanar;
	//def.format.video.eCompressionFormat = OMX_VIDEO_CodingUnused;
	//def.format.video.eColorFormat = OMX_COLOR_FormatBRCMEGL;//OMX_COLOR_Format24bitBGR888; //OMX_COLOR_Format32bitABGR8888;//OMX_COLOR_FormatYUV420PackedPlanar;
	//Must be manually defined to ensure sufficient size if stride needs to be rounded up to multiple of 32.
	def.nBufferSize = def.format.video.nStride * def.format.video.nSliceHeight;
	//We allocate 1 input buffers.
	def.nBufferCountActual = 1;

	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->image_encode),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_SetParameter failed for input format definition.");

	//Set the output format of the encoder
	OMX_VIDEO_PARAM_PORTFORMATTYPE format = { };
	format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
	format.nVersion.nVersion = OMX_VERSION;
	format.nPortIndex = 201;
	format.eCompressionFormat = OMX_VIDEO_CodingAVC;

	ret = OMX_SetParameter(ILC_GET_HANDLE(_this->image_encode),
			OMX_IndexParamVideoPortFormat, &format);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_SetParameter failed for setting encoder output format.");

	ret = ilclient_change_component_state(_this->image_encode, OMX_StateIdle);
	CHECKED(ret != 0, "ILClient failed to change encoder to idle state.");
	ret = ilclient_enable_port_buffers(_this->image_encode, 200, NULL, NULL,
			NULL);
	CHECKED(ret != 0, "ILClient failed to enable input buffers.");
	ret = ilclient_enable_port_buffers(_this->image_encode, 201, NULL, NULL,
			NULL);
	CHECKED(ret != 0, "ILClient failed to enable output buffers.");
	ret = ilclient_change_component_state(_this->image_encode,
			OMX_StateExecuting);
	CHECKED(ret != 0, "ILClient failed to change encoder to executing stage.");
#endif
}
OMXJPEG_FN_DEFINE(void, jpeg_finish_compress, (j_compress_ptr cinfo)) {
	omx_jpeg_compress_private *_this =
			(omx_jpeg_compress_private*) cinfo->master;

	_this->in_buffer = ilclient_get_input_buffer(_this->image_encode, 340, 1);
	memcpy(_this->in_buffer->pBuffer, cinfo->client_data,
			cinfo->image_width * 4 * cinfo->image_height);
	_this->in_buffer->nFilledLen = _this->in_buffer->nAllocLen;
	OMX_EmptyThisBuffer(ILC_GET_HANDLE(_this->image_encode), _this->in_buffer);

	*_this->outsize = 0;
	do {
		_this->out_buffer = ilclient_get_output_buffer(_this->image_encode, 341,
				1);
		_this->fill_buffer_done = FALSE;
		_this->out_buffer->nFilledLen = 0;
		OMX_FillThisBuffer(ILC_GET_HANDLE(_this->image_encode),
				_this->out_buffer);

		pthread_mutex_lock(&_this->mutex);
		if (_this->fill_buffer_done != TRUE) {
			pthread_cond_wait(&_this->cond, &_this->mutex);
		}
		pthread_mutex_unlock(&_this->mutex);

		int size = MIN(_this->out_buffer->nFilledLen,
				cinfo->dest->free_in_buffer);
		if (size) {
			memcpy(cinfo->dest->next_output_byte, _this->out_buffer->pBuffer,
					_this->out_buffer->nFilledLen);
			cinfo->dest->next_output_byte += _this->out_buffer->nFilledLen;
			cinfo->dest->free_in_buffer -= _this->out_buffer->nFilledLen;
			*_this->outsize += _this->out_buffer->nFilledLen;
		}
	} while (!(_this->out_buffer->nFlags & OMX_BUFFERFLAG_ENDOFFRAME));

	//printf("jpeg_finish_compress done %d\n", *_this->outsize);
	return;
}
