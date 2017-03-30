/**
 * picam360-driver @ picam360 project
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <limits.h>

#include <iostream>
#include <string>

#include "rtpsession.h"
#include "rtpudpv4transmitter.h"
#include "rtpipv4address.h"
#include "rtpsessionparams.h"
#include "rtperrors.h"
#include "rtplibraryversion.h"
#include "rtppacket.h"

#include "rtp.h"

using namespace jrtplib;

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static bool lg_receive_run = false;
pthread_t lg_receive_thread;
static RTPSession lg_sess;
static pthread_mutex_t lg_mlock = PTHREAD_MUTEX_INITIALIZER;
static int lg_record_fd = -1;
static int lg_load_fd = -1;
pthread_t lg_load_thread;

static RTP_CALLBACK lg_callback = NULL;

void rtp_set_callback(RTP_CALLBACK callback) {
	lg_callback = callback;
}

static void checkerror(int rtperr) {
	if (rtperr < 0) {
		std::cout << "ERROR: " << RTPGetErrorString(rtperr) << std::endl;
		exit(-1);
	}
}

int rtp_sendpacket(unsigned char *data, int data_len, int pt) {
	int status;
	pthread_mutex_lock(&lg_mlock);
	{
		static struct timeval last_time = { };
		struct timeval time = { };
		gettimeofday(&time, NULL);
		struct timeval diff;
		timersub(&time, &last_time, &diff);
		int diff_nsec = diff.tv_sec * 1000000 + (float) diff.tv_usec;
		if (diff_nsec == 0) {
			diff_nsec = 1;
		}
		status = lg_sess.SendPacket(data, data_len, pt, false, diff_nsec);
		checkerror(status);
		last_time = time;
	}
	pthread_mutex_unlock(&lg_mlock);
	return 0;
}

static void *receive_thread_func(void* arg) {
	int status;
	while (lg_receive_run) {
		lg_sess.BeginDataAccess();

		// check incoming packets
		if (lg_sess.GotoFirstSourceWithData()) {
			do {
				RTPPacket *pack;

				while ((pack = lg_sess.GetNextPacket()) != NULL) {
					if (lg_callback && lg_load_fd < 0) {
						lg_callback(pack->GetPayloadData(),
								pack->GetPayloadLength(),
								pack->GetPayloadType());
					}
					if (lg_record_fd > 0) {
						unsigned short len = pack->GetPacketLength();
						unsigned char len_bytes[2];
						for (int i = 0; i < 2; i++) {
							len_bytes[i] = (len >> (8 * i)) & 0xFF;
						}
						write(lg_record_fd, len_bytes, 2);
						write(lg_record_fd, pack->GetPacketData(), len);
					}

					lg_sess.DeletePacket(pack);
				}
			} while (lg_sess.GotoNextSourceWithData());
		}

		lg_sess.EndDataAccess();

#ifndef RTP_SUPPORT_THREAD
		status = lg_sess.Poll();
		checkerror(status);
#endif // RTP_SUPPORT_THREAD
	}
	return NULL;
}

static void *load_thread_func(void* arg) {
	unsigned char buff[RTP_MAXPAYLOADSIZE + sizeof(struct RTPHeader)];
	while (lg_load_fd >= 0) {
		int read_len;
		struct RTPHeader *header = (struct RTPHeader *) buff;
		read_len = read(lg_load_fd, buff, 2);
		if (read_len != 2) {
			//error
			break;
		}
		unsigned short len = 0;
		for (int i = 0; i < 2; i++) {
			len += (buff[i] << (8 * i));
		}
		if (len > sizeof(buff)) {
			//error
			break;
		}
		read_len = read(lg_load_fd, buff, len);
		if (read_len != len) {
			//error
			break;
		}

		if (lg_callback) {
			lg_callback(buff + sizeof(struct RTPHeader),
					len - sizeof(struct RTPHeader), header->payloadtype);
		}

		//lg_sess.DeletePacket(pack);
	}
	return NULL;
}

static bool is_init = false;
int init_rtp(unsigned short portbase, char *destip_str,
		unsigned short destport) {
	if (is_init) {
		return -1;
	}
	is_init = true;

	uint32_t destip;
	int status;

	destip = inet_addr(destip_str);
	if (destip == INADDR_NONE) {
		std::cerr << "Bad IP address specified" << std::endl;
		return -1;
	}

	// The inet_addr function returns a value in network byte order, but
	// we need the IP address in host byte order, so we use a call to
	// ntohl
	destip = ntohl(destip);

	RTPUDPv4TransmissionParams transparams;
	RTPSessionParams sessparams;

	sessparams.SetOwnTimestampUnit(1.0 / 1E6);	//micro sec
	sessparams.SetMaximumPacketSize(
	RTP_MAXPAYLOADSIZE + sizeof(struct RTPHeader));
	sessparams.SetAcceptOwnPackets(true);
	transparams.SetPortbase(portbase);

	status = lg_sess.Create(sessparams, &transparams);
	checkerror(status);

	RTPIPv4Address addr(destip, destport);

	status = lg_sess.AddDestination(addr);
	checkerror(status);

	lg_receive_run = true;
	pthread_create(&lg_receive_thread, NULL, receive_thread_func, (void*) NULL);

	return 0;
}

int deinit_rtp() {
	if (!is_init) {
		return -1;
	}
	lg_receive_run = false;
	pthread_join(lg_receive_thread, NULL);
	lg_sess.BYEDestroy(RTPTime(10, 0), 0, 0);
	is_init = false;
	return 0;
}

void rtp_start_recording(char *path) {
	rtp_stop_recording();
	lg_record_fd = open(path, O_CREAT | O_WRONLY | O_TRUNC);
}

void rtp_stop_recording() {
	if (lg_record_fd > 0) {
		close(lg_record_fd);
		lg_record_fd = -1;
	}
}

void rtp_start_loading(char *path) {
	rtp_stop_loading();
	lg_load_fd = open(path, O_RDONLY);
	pthread_create(&lg_load_thread, NULL, load_thread_func, (void*) NULL);
}

void rtp_stop_loading() {
	if (lg_load_fd > 0) {
		close(lg_load_fd);
		lg_load_fd = -1;
		pthread_join(lg_load_thread, NULL);
	}
}
