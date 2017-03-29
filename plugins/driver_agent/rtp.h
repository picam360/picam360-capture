#pragma once

#define RTP_MAXPACKETSIZE 1400

#ifdef __cplusplus
extern "C" {
#endif

int init_rtp();
int deinit_rtp();
int rtp_sendpacket(char *data, int data_len, int pt);

typedef void (*RTP_CALLBACK)(char *data, int data_len, int pt);
void rtp_set_callkback(RTP_CALLBACK callback);

#ifdef __cplusplus
}
#endif
