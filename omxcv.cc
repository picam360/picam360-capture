/**
 * @file omxcv.cpp
 * @brief Routines to perform hardware accelerated H.264 encoding on the RPi.
 */

#include "omxcv.h"
#include "omxcv-impl.h"
using namespace omxcv;

using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>

#define TIMEDIFF(start) (duration_cast<milliseconds>(steady_clock::now() - start).count())

//#ifdef ENABLE_NEON
//extern "C" void omxcv_bgr2rgb_neon(const unsigned char *src, unsigned char *dst, int n);
//#endif
//
///**
// * Perform the BGR2RGB conversion.
// * @param [in] src The source buffer.
// * @param [in] dst The destination buffer.
// * @param [in] stride The stride of the image.
// */
//void BGR2RGB(const cv::Mat &src, uint8_t *dst, int stride) {
//#ifdef ENABLE_NEON
//	for (int i = 0; i < src.rows; i++) {
//		const uint8_t *buffer = src.ptr<const uint8_t>(i);
//		omxcv_bgr2rgb_neon(buffer, dst+stride*i, src.cols);
//	}
//#else
//	cv::Mat omat(src.rows, src.cols, CV_8UC3, dst, stride);
//	cv::cvtColor(src, omat, CV_BGR2RGB);
//#endif
//}

/**
 * Constructor.
 * @param [in] name The file to save to.
 * @param [in] width The video width.
 * @param [in] height The video height.
 * @param [in] bitrate The bitrate, in Kbps.
 * @param [in] fpsnum The FPS numerator.
 * @param [in] fpsden The FPS denominator.
 */
OmxCvImpl::OmxCvImpl(const char *name, int width, int height, int bitrate,
		int fpsnum, int fpsden, OMXCV_CALLBACK callback, void *user_data) :
		m_width(width), m_height(height), m_stride(((width + 31) & ~31) * 3), m_bitrate(
				bitrate), m_filename(name), m_stop { false }, m_callback(
				callback), m_user_data(user_data) {
	int ret;
	bcm_host_init();

	std::string extention = m_filename.substr(m_filename.find_last_of(".") + 1);
	std::transform(extention.cbegin(), extention.cend(), extention.begin(),
			tolower);
	if (extention == "jpeg" || extention == "jpg") {
		mcodec_type = JPEG;
	} else if (extention == "mjpeg" || extention == "mjpg") {
		mcodec_type = MJPEG;
	} else {
		mcodec_type = H264;
	}

	if (fpsden <= 0 || fpsnum <= 0) {
		fpsden = 1;
		fpsnum = 25;
	}
	m_fpsnum = fpsnum;
	m_fpsden = fpsden;

	//Initialise OpenMAX and the IL client.
	CHECKED(OMX_Init() != OMX_ErrorNone, "OMX_Init failed.");
	m_ilclient = ilclient_init();
	CHECKED(m_ilclient == NULL, "ILClient initialisation failed.");

	ret = ilclient_create_component(m_ilclient, &m_encoder_component,
			(char*) "video_encode",
			(ILCLIENT_CREATE_FLAGS_T)(
					ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_INPUT_BUFFERS
							| ILCLIENT_ENABLE_OUTPUT_BUFFERS));
	CHECKED(ret != 0, "ILCient video_encode component creation failed.");

	//Set input definition to the encoder
	OMX_PARAM_PORTDEFINITIONTYPE def = { };
	def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	def.nVersion.nVersion = OMX_VERSION;
	def.nPortIndex = OMX_ENCODE_PORT_IN;
	ret = OMX_GetParameter(ILC_GET_HANDLE(m_encoder_component),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_GetParameter failed for encode port in.");

	def.format.video.nFrameWidth = m_width;
	def.format.video.nFrameHeight = m_height;
	if (mcodec_type == JPEG || mcodec_type == MJPEG) {
		def.format.video.xFramerate = 10 << 16;
	} else {
		def.format.video.xFramerate = 30 << 16;
	}
	//Must be a multiple of 16
	def.format.video.nSliceHeight = (m_height + 15) & ~15;
	//Must be a multiple of 32
	def.format.video.nStride = m_stride;
	def.format.video.eColorFormat = OMX_COLOR_Format24bitBGR888; //OMX_COLOR_Format32bitABGR8888;//OMX_COLOR_FormatYUV420PackedPlanar;
	//Must be manually defined to ensure sufficient size if stride needs to be rounded up to multiple of 32.
	def.nBufferSize = def.format.video.nStride * def.format.video.nSliceHeight;
	//We allocate 1 input buffers.
	def.nBufferCountActual = 1;

	ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
			OMX_IndexParamPortDefinition, &def);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_SetParameter failed for input format definition.");

	//Set the output format of the encoder
	OMX_VIDEO_PARAM_PORTFORMATTYPE format = { };
	format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
	format.nVersion.nVersion = OMX_VERSION;
	format.nPortIndex = OMX_ENCODE_PORT_OUT;
	if (mcodec_type == JPEG || mcodec_type == MJPEG) {
		format.eCompressionFormat = OMX_VIDEO_CodingMJPEG;
	} else {
		format.eCompressionFormat = OMX_VIDEO_CodingAVC;
	}

	ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
			OMX_IndexParamVideoPortFormat, &format);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_SetParameter failed for setting encoder output format.");

	//Set the encoding bitrate
	OMX_VIDEO_PARAM_BITRATETYPE bitrate_type = { };
	bitrate_type.nSize = sizeof(OMX_VIDEO_PARAM_BITRATETYPE);
	bitrate_type.nVersion.nVersion = OMX_VERSION;
	bitrate_type.eControlRate = OMX_Video_ControlRateVariable;
	if (mcodec_type == JPEG || mcodec_type == MJPEG) {
		bitrate_type.nTargetBitrate = 8 * 1000 * 1000;
	} else {
		bitrate_type.nTargetBitrate = bitrate * 1000;
	}
	bitrate_type.nPortIndex = OMX_ENCODE_PORT_OUT;
	ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
			OMX_IndexParamVideoBitrate, &bitrate_type);
	CHECKED(ret != OMX_ErrorNone,
			"OMX_SetParameter failed for setting encoder bitrate.");

	if (format.eCompressionFormat == OMX_VIDEO_CodingAVC) {
		//Set the output profile level of the encoder
		OMX_VIDEO_PARAM_PROFILELEVELTYPE profileLevel = { }; // OMX_IndexParamVideoProfileLevelCurrent
		profileLevel.nSize = sizeof(OMX_VIDEO_PARAM_PROFILELEVELTYPE);
		profileLevel.nVersion.nVersion = OMX_VERSION;
		profileLevel.nPortIndex = OMX_ENCODE_PORT_OUT;
		profileLevel.eProfile = OMX_VIDEO_AVCProfileBaseline;
		profileLevel.eLevel = OMX_VIDEO_AVCLevel31;
		ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
				OMX_IndexParamVideoProfileLevelCurrent, &profileLevel);
		CHECKED(ret != 0,
				"OMX_SetParameter failed for setting avc profile & level.");

		//I think this decreases the chance of NALUs being split across buffers.
		/*
		 OMX_CONFIG_BOOLEANTYPE frg = {};
		 frg.nSize = sizeof(OMX_CONFIG_BOOLEANTYPE);
		 frg.nVersion.nVersion = OMX_VERSION;
		 frg.bEnabled = OMX_TRUE;
		 ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
		 OMX_IndexConfigMinimiseFragmentation, &frg);
		 CHECKED(ret != 0, "OMX_SetParameter failed for setting fragmentation minimisation.");
		 */

		//We want at most one NAL per output buffer that we receive.
		OMX_CONFIG_BOOLEANTYPE nal = { };
		nal.nSize = sizeof(OMX_CONFIG_BOOLEANTYPE);
		nal.nVersion.nVersion = OMX_VERSION;
		nal.bEnabled = OMX_TRUE;
		ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
				OMX_IndexParamBrcmNALSSeparate, &nal);
		CHECKED(ret != 0,
				"OMX_SetParameter failed for setting separate NALUs.");

		//We want the encoder to write the NALU length instead start codes.
		OMX_NALSTREAMFORMATTYPE nal2 = { };
		nal2.nSize = sizeof(OMX_NALSTREAMFORMATTYPE);
		nal2.nVersion.nVersion = OMX_VERSION;
		nal2.nPortIndex = OMX_ENCODE_PORT_OUT;
		nal2.eNaluFormat = OMX_NaluFormatFourByteInterleaveLength;
		ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
				(OMX_INDEXTYPE) OMX_IndexParamNalStreamFormatSelect, &nal2);
		CHECKED(ret != 0, "OMX_SetParameter failed for setting NALU format.");

		//Inline SPS/PPS
		OMX_CONFIG_PORTBOOLEANTYPE inlineHeader = { };
		inlineHeader.nSize = sizeof(OMX_CONFIG_PORTBOOLEANTYPE);
		inlineHeader.nVersion.nVersion = OMX_VERSION;
		inlineHeader.nPortIndex = OMX_ENCODE_PORT_OUT;
		inlineHeader.bEnabled = OMX_TRUE;
		ret = OMX_SetParameter(ILC_GET_HANDLE(m_encoder_component),
				OMX_IndexParamBrcmVideoAVCInlineHeaderEnable, &inlineHeader);
		CHECKED(ret != 0,
				"OMX_SetParameter failed for setting inline sps/pps.");
	}

	ret = ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
	CHECKED(ret != 0, "ILClient failed to change encoder to idle state.");

	OMX_SendCommand(ILC_GET_HANDLE(m_encoder_component), OMX_CommandPortEnable,
	OMX_ENCODE_PORT_IN, NULL);
	ilclient_wait_for_event(m_encoder_component, OMX_EventCmdComplete,
			OMX_CommandPortEnable, 1,
			OMX_ENCODE_PORT_IN, 1, 0, 1000);
	OMX_SendCommand(ILC_GET_HANDLE(m_encoder_component), OMX_CommandPortEnable,
	OMX_ENCODE_PORT_OUT, NULL);
	ilclient_wait_for_event(m_encoder_component, OMX_EventCmdComplete,
			OMX_CommandPortEnable, 1,
			OMX_ENCODE_PORT_OUT, 1, 0, 1000);

	ret = OMX_AllocateBuffer(ILC_GET_HANDLE(m_encoder_component), &input_buffer,
	OMX_ENCODE_PORT_IN, NULL, def.nBufferSize);
	CHECKED(ret != 0, "OMX_AllocateBuffer input");

	ret = OMX_AllocateBuffer(ILC_GET_HANDLE(m_encoder_component),
			&output_buffer,
			OMX_ENCODE_PORT_OUT, NULL, def.nBufferSize);
	CHECKED(ret != 0, "OMX_AllocateBuffer output");

	ret = ilclient_change_component_state(m_encoder_component,
			OMX_StateExecuting);
	CHECKED(ret != 0, "ILClient failed to change encoder to executing stage.");

	if (m_callback || mcodec_type == JPEG) {
	} else {
		m_ofstream.open(m_filename, std::ios::out);
	}

	//Start the worker thread for dumping the encoded data
	m_input_worker = std::thread(&OmxCvImpl::input_worker, this);

	//for jpeg
	image_buff = NULL;
	image_buff_size = 0;
	image_buff_cur = 0;
	image_start = -1;
	data_len_total = 0;
	marker = 0;
	soicount = 0;
}

/**
 * Destructor.
 * @return Return_Description
 */
OmxCvImpl::~OmxCvImpl() {
	m_stop = true;
	m_input_signaller.notify_one();
	m_input_worker.join();

	//Teardown similar to hello_encode
	ilclient_change_component_state(m_encoder_component, OMX_StateIdle);

	OMX_FreeBuffer(ILC_GET_HANDLE(m_encoder_component), OMX_ENCODE_PORT_IN,
			input_buffer);
	OMX_FreeBuffer(ILC_GET_HANDLE(m_encoder_component), OMX_ENCODE_PORT_OUT,
			output_buffer);

	//ilclient_change_component_state(m_encoder_component, OMX_StateIdle);
	ilclient_change_component_state(m_encoder_component, OMX_StateLoaded);

	COMPONENT_T *list[] = { m_encoder_component, NULL };
	ilclient_cleanup_components(list);
	ilclient_destroy(m_ilclient);
}

/**
 * Input encoding routine.
 */
void OmxCvImpl::input_worker() {
	std::unique_lock < std::mutex > lock(m_input_mutex);

	while (true) {
		m_input_signaller.wait(lock,
				[this] {return m_stop || m_input_queue.size() > 0;});
		if (m_stop) {
			if (m_input_queue.size() > 0) {
				printf(
						"Stop acknowledged but need to flush the input buffer (%d)...\n",
						m_input_queue.size());
			} else {
				break;
			}
		}

//auto proc_start = steady_clock::now();
		std::pair<OMX_BUFFERHEADERTYPE *, int64_t> frame =
				m_input_queue.front();
		m_input_queue.pop_front();
		lock.unlock();

//auto conv_start = steady_clock::now();
//static int framecounter = 0;

		OMX_EmptyThisBuffer(ILC_GET_HANDLE(m_encoder_component), frame.first);
//fflush(stdout);
//printf("Encoding time (ms): %d [%d]\r", (int)TIMEDIFF(conv_start), ++framecounter);
		do {
			output_buffer->nFilledLen = 0; //I don't think this is necessary, but whatever.
			OMX_FillThisBuffer(ILC_GET_HANDLE(m_encoder_component),
					output_buffer);
		} while (!write_data(output_buffer, frame.second));

		lock.lock();
//printf("Total processing time (ms): %d\n", (int)TIMEDIFF(proc_start));
	}

	//Needed because we call ilclient_get_output_buffer last.
	//Otherwise ilclient waits forever for the buffer to be filled.
	OMX_FillThisBuffer(ILC_GET_HANDLE(m_encoder_component), output_buffer);
}

/**
 * Output muxing routine.
 * @param [in] out Buffer to be saved.
 * @param [in] timestamp Timestamp of this buffer.
 * @return true if buffer was saved.
 */
bool OmxCvImpl::write_data(OMX_BUFFERHEADERTYPE *out, int64_t timestamp) {

	if (out->nFilledLen != 0) {
		if (m_callback) {
			unsigned char *buff = out->pBuffer;
			int data_len = out->nFilledLen;
			m_callback(out->pBuffer, data_len, m_user_data);
		} else if (mcodec_type == JPEG) {
			unsigned char *buff = out->pBuffer;
			int data_len = out->nFilledLen;
			for (int i = 0; i < data_len; i++) {
				if (marker) {
					marker = 0;
					if (buff[i] == 0xd8) { //SOI
						if (soicount == 0) {
							image_start = data_len_total + (i - 1);
						}
						soicount++;
					}
					if (buff[i] == 0xd9 && image_start >= 0) { //EOI
						soicount--;
						if (soicount == 0) {
							int image_size = (data_len_total - image_start)
									+ (i + 1);
							if (image_size > image_buff_size) { //just allocate image buffer
								image_buff_size = image_size * 2;
								if (image_buff != NULL) {
									free(image_buff);
								}
							} else {
								memcpy(image_buff + image_buff_cur, buff,
										i + 1);
								int fd = open(m_filename.c_str(),
										O_WRONLY | O_CREAT, 0666);
								if (fd != -1) {
									write(fd, (const char*) image_buff,
											image_size);
									close(fd);
								}
								free(image_buff);
//printf("save frame\n");
							}
							image_buff_cur = 0;
							image_buff = (unsigned char*) malloc(
									image_buff_size);
							image_start = -1;
						}
					}
				} else if (buff[i] == 0xff) {
					marker = 1;
				}
			}
			if (image_buff != NULL && image_start >= 0) {
				if (image_buff_cur + data_len > image_buff_size) { //exceed buffer size
					free(image_buff);
					image_buff = NULL;
					image_buff_size = 0;
					image_buff_cur = 0;
				} else if (image_start > data_len_total) { //soi
					memcpy(image_buff, buff + (image_start - data_len_total),
							data_len - (image_start - data_len_total));
					image_buff_cur = data_len - (image_start - data_len_total);
				} else {
					memcpy(image_buff + image_buff_cur, buff, data_len);
					image_buff_cur += data_len;
				}
			}
			data_len_total += data_len;
		} else {
			//printf("write data : %d\n", (int)out->nFilledLen);
			m_ofstream.write((const char*) out->pBuffer, (int) out->nFilledLen);
		}
		return true;
	} else {
		//printf("write data : return false\n");
		return true;
	}
	out->nFilledLen = 0;
}

/**
 * Enqueue video to be encoded.
 * @param [in] mat The mat to be encoded.
 * @return true iff enqueued.
 */
bool OmxCvImpl::process(const unsigned char *in_data) {
	if (m_input_queue.size() > 0) {
		return false;
	}
	auto now = steady_clock::now();
	memcpy(input_buffer->pBuffer, in_data, m_stride * m_height);
	//BGR2RGB(mat, in->pBuffer, m_stride);
	input_buffer->nFilledLen = input_buffer->nAllocLen;
	if (m_frame_count == 0) {
		input_buffer->nFlags = OMX_BUFFERFLAG_STARTTIME;
	} else {
		input_buffer->nFlags = OMX_BUFFERFLAG_TIME_UNKNOWN;
	}
	input_buffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;

	std::unique_lock < std::mutex > lock(m_input_mutex);
	if (m_frame_count++ == 0) {
		m_frame_start = now;
	}
	m_input_queue.push_back(
			std::pair<OMX_BUFFERHEADERTYPE *, int64_t>(input_buffer,
					duration_cast < milliseconds
							> (now - m_frame_start).count()));
	lock.unlock();
	m_input_signaller.notify_one();
	return true;
}

/**
 * Constructor for our wrapper.
 * @param [in] name The file to save to.
 * @param [in] width The video width.
 * @param [in] height The video height.
 * @param [in] bitrate The bitrate, in Kbps.
 * @param [in] fpsnum The FPS numerator.
 * @param [in] fpsden The FPS denominator.
 */
OmxCv::OmxCv(const char *name, int width, int height, int bitrate, int fpsnum,
		int fpsden, OMXCV_CALLBACK callback, void *user_data) {
	m_impl = new OmxCvImpl(name, width, height, bitrate, fpsnum, fpsden,
			callback, user_data);
}

/**
 * Wrapper destructor.
 */
OmxCv::~OmxCv() {
	delete m_impl;
}

/**
 * Encode image.
 * @param [in] in Image to be encoded.
 * @return true iff the image was encoded.
 */
bool OmxCv::Encode(const unsigned char *in_data) {
	return m_impl->process(in_data);
}
