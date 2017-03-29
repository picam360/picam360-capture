#pragma once

#define RTP_MAXPAYLOADSIZE (4*1024)

#ifdef __cplusplus
extern "C" {
#endif

int init_rtp(unsigned short portbase, char *destip_str, unsigned short destport);
int deinit_rtp();
int rtp_sendpacket(unsigned char *data, int data_len, int pt);
void rtp_start_recording(char *path);
void rtp_stop_recording();

typedef void (*RTP_CALLBACK)(unsigned char *data, int data_len, int pt);
void rtp_set_callback(RTP_CALLBACK callback);

#ifdef __cplusplus
}
#endif
