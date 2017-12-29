#pragma once

#define RTP_MAXPAYLOADSIZE (8*1024-12-8)

#ifdef __cplusplus
extern "C" {
#endif

enum RTP_SOCKET_TYPE {
	RTP_SOCKET_TYPE_NONE, RTP_SOCKET_TYPE_UDP, RTP_SOCKET_TYPE_TCP, RTP_SOCKET_TYPE_FIFO
};

typedef void (*RTP_LOADING_CALLBACK)(void *user_data, int ret);
typedef struct _RTP_T RTP_T;

RTP_T *create_rtp(unsigned short rx_port, enum RTP_SOCKET_TYPE rx_socket_type, char *tx_ip, unsigned short tx_port, enum RTP_SOCKET_TYPE tx_socket_type, float bandwidth_limit);
int delete_rtp(RTP_T **_this_p);
int rtp_set_buffer_size(RTP_T *_this, int rx_buffer_size, int tx_buffer_size);
int rtp_sendpacket(RTP_T *_this, const unsigned char *data, int data_len, int pt);
void rtp_flush(RTP_T *_this);
void rtp_start_recording(RTP_T *_this, char *path);
void rtp_stop_recording(RTP_T *_this);
bool rtp_is_recording(RTP_T *_this, char **path);
bool rtp_start_loading(RTP_T *_this, char *path, bool auto_play, bool is_looping, RTP_LOADING_CALLBACK callback, void *userdata);
void rtp_increment_loading(RTP_T *_this, int elapsed_usec);
void rtp_set_play_speed(RTP_T *_this, float play_speed);
void rtp_stop_loading(RTP_T *_this);
bool rtp_is_loading(RTP_T *_this, char **path);
float rtp_get_bandwidth(RTP_T *_this);
void rtp_set_auto_play(RTP_T *_this, bool value);
void rtp_set_is_looping(RTP_T *_this, bool value);

const char *rtp_get_rtp_socket_type_str(enum RTP_SOCKET_TYPE type);
enum RTP_SOCKET_TYPE rtp_get_rtp_socket_type(const char *type_str);

typedef void (*RTP_CALLBACK)(unsigned char *data, unsigned int data_len, unsigned char pt, unsigned int seq_num);
void rtp_set_callback(RTP_T *_this, RTP_CALLBACK callback);

#ifdef __cplusplus
}
#endif
