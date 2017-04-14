#pragma once

#define RTP_MAXPAYLOADSIZE (8*1024-12)

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*RTP_LOADING_CALLBACK)(void *user_data, int ret);

int init_rtp(unsigned short portbase, char *destip_str,
		unsigned short destport);
int deinit_rtp();
int rtp_sendpacket(unsigned char *data, int data_len, int pt);
void rtp_start_recording(char *path);
void rtp_stop_recording();
bool rtp_is_recording(char **path);
bool rtp_start_loading(char *path, bool auto_play, bool is_looping,
		RTP_LOADING_CALLBACK callback, void *userdata);
void rtp_increment_loading(int elapsed_usec);
void rtp_stop_loading();
bool rtp_is_loading(char **path);
float rtp_get_bandwidth();

typedef void (*RTP_CALLBACK)(unsigned char *data, unsigned int data_len,
		unsigned char pt, unsigned int seq_num);
void rtp_set_callback(RTP_CALLBACK callback);

#ifdef __cplusplus
}
#endif
