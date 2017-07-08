#pragma once

#define RTCP_MAXPAYLOADSIZE (8*1024-12)

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*RTCP_LOADING_CALLBACK)(void *user_data, int ret);

int init_rtcp(unsigned short portbase, char *destip_str, unsigned short destport,
		float bandwidth_limit);
int deinit_rtcp();
int rtcp_sendpacket(unsigned char *data, int data_len, int pt);
void rtcp_start_recording(char *path);
void rtcp_stop_recording();
bool rtcp_is_recording(char **path);
bool rtcp_start_loading(char *path, bool auto_play, bool is_looping,
		RTCP_LOADING_CALLBACK callback, void *userdata);
void rtcp_increment_loading(int elapsed_usec);
void rtcp_stop_loading();
bool rtcp_is_loading(char **path);
float rtcp_get_bandwidth();
void rtcp_set_auto_play(bool value);
void rtcp_set_is_looping(bool value);

typedef void (*RTCP_CALLBACK)(unsigned char *data, unsigned int data_len,
		unsigned char pt, unsigned int seq_num);
void rtcp_set_callback(RTCP_CALLBACK callback);

#ifdef __cplusplus
}
#endif
