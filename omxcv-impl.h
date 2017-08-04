/**
 * @file omxcv-impl.cpp
 * @brief Actual implementation class
 */

#ifndef __OMXCV_IMPL_H
#define __OMXCV_IMPL_H

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <utility>
#include <fstream>

//#include <opencv2/opencv.hpp>

//Sigh
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/avutil.h>
#include <libavutil/mathematics.h>
#include <libavformat/avio.h>

//Without the right struct packing the buffers will be screwed...
//nFlags won't be set correctly...
#pragma pack(4)
#include <bcm_host.h>
#include <ilclient.h>
//For OMX_IndexParamNalStreamFormatSelect
#include <IL/OMX_Broadcom.h>
#pragma pack()
}

//Determine what frame allocation routine to use
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55,28,1)
#define OMXCV_AV_FRAME_ALLOC av_frame_alloc
#define OMXCV_AV_FRAME_FREE av_frame_free
#else
#define OMXCV_AV_FRAME_ALLOC avcodec_alloc_frame
#define OMXCV_AV_FRAME_FREE avcodec_free_frame
#endif

#define OMX_ENCODE_PORT_IN  200
#define OMX_ENCODE_PORT_OUT 201

#define OMX_JPEG_PORT_IN  340
#define OMX_JPEG_PORT_OUT 341

//The maximum size of a NALU. We'll just assume 512 KB.
#define MAX_NALU_SIZE (512*1024)

#define CHECKED(c, v) if ((c)) throw std::invalid_argument(v)

extern void BGR2RGB(const cv::Mat &src, uint8_t *dst, int stride);

namespace omxcv {
enum CODEC_TYPE {
	H264, MJPEG, JPEG
};
/**
 * Our implementation class of the encoder.
 */
class OmxCvImpl {
public:
	OmxCvImpl(const char *name, int width, int height, int bitrate, int fpsnum =
			-1, int fpsden = -1, OMXCV_CALLBACK callback = NULL,
			void *user_data = NULL);
	virtual ~OmxCvImpl();

	bool process(const unsigned char *in_data);

private:
	int m_width, m_height, m_stride, m_bitrate, m_fpsnum, m_fpsden;
	OMXCV_CALLBACK m_callback;
	void *m_user_data;

	enum CODEC_TYPE m_codec_type;
	std::string m_filename;
	std::ofstream m_ofstream;

	std::condition_variable m_input_signaller;
	std::deque<std::pair<OMX_BUFFERHEADERTYPE *, int64_t>> m_input_queue;
	std::thread m_input_worker;
	std::mutex m_input_mutex;
	std::atomic<bool> m_stop;

	/** The OpenMAX IL client **/
	ILCLIENT_T *m_ilclient;
	COMPONENT_T *m_encoder_component;

	std::chrono::steady_clock::time_point m_frame_start;
	int m_frame_count;

	//for jpeg
	unsigned char *image_buff;
	int image_buff_size;
	int image_buff_cur;
	int image_start;
	int data_len_total;
	int marker;
	int soicount;
	bool first_packet;
	OMX_BUFFERHEADERTYPE *output_buffer;

	void input_worker();
	bool write_data(OMX_BUFFERHEADERTYPE *out, int64_t timestamp);
	static void my_fill_buffer_done(void* data, COMPONENT_T* comp);
};

class OmxCvJpegImpl {
public:
	OmxCvJpegImpl(int width, int height, int quality = 90);
	virtual ~OmxCvJpegImpl();

	bool process(const char *filename, const unsigned char *in_data);
private:
	int m_width, m_height, m_stride, m_quality;

	std::condition_variable m_input_signaller;
	std::deque<std::pair<OMX_BUFFERHEADERTYPE *, std::string>> m_input_queue;
	std::thread m_input_worker;
	std::mutex m_input_mutex;
	std::atomic<bool> m_stop;

	ILCLIENT_T *m_ilclient;
	COMPONENT_T *m_encoder_component;

	void input_worker();
};
}

#endif // __OMXCV_IMPL_H
