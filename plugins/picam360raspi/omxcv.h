/**
 * @file omxcv.cpp
 * @brief Header file for the library
 */

#ifndef __OMXCV_H
#define __OMXCV_H

#include <opencv2/opencv.hpp>

typedef void (*OMXCV_CALLBACK)(unsigned char *data, unsigned int data_len,
		void *frame_data, void *user_data);

namespace omxcv {
    /* Forward declaration of our H.264 implementation. */
    class OmxCvImpl;
    /* Forward delaration of our JPEG implementation. */
    class OmxCvJpegImpl;

    /**
     * Real-time OpenMAX H.264 encoder for the Raspberry Pi/OpenCV.
     */
    class OmxCv {
        public:
            OmxCv(const char *name, int width, int height, int bitrate=3000, int fpsnum=25, int fpsden=1, OMXCV_CALLBACK callback=NULL, void *user_data=NULL);
            bool Encode(const unsigned char *in_data, void *frame_data);
            virtual ~OmxCv();
        private:
            OmxCvImpl *m_impl;
    };
    
    /**
     * Real-time OpenMAX JPEG encoder for the Raspberry Pi/OpenCV.
     */
     class OmxCvJpeg {
         public:
            OmxCvJpeg(int width, int height, int quality=90);
            bool Encode(const char *filename, const unsigned char *in_data);
            virtual ~OmxCvJpeg();
         private:
            OmxCvJpegImpl *m_impl;
            int m_width, m_height, m_quality;
     };
}

#endif /* _OMXCV_H */
