#ifndef H265_ENCODER_H
#define H265_ENCODER_H

typedef void (*H265_STREAM_CALLBACK)(unsigned char *data, unsigned int data_len, void *frame_data, void *user_data);

typedef struct _h265_encoder h265_encoder;

h265_encoder *h265_create_encoder(const int width, const int height, int bitrate_kbps, int fps, H265_STREAM_CALLBACK callback, void *user_data);
void h265_delete_encoder(h265_encoder * _this);
void h265_add_frame(h265_encoder *_this, const unsigned char *in_data, void *frame_data);

#endif

