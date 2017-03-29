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

#include "rtp.h"

using namespace jrtplib;

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static bool lg_receive_run = false;
pthread_t lg_receive_thread;
static RTPSession lg_sess;
static pthread_mutex_t lg_mlock = PTHREAD_MUTEX_INITIALIZER;

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

int rtp_sendpacket(char *buff, int buff_len, int pt) {
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
		status = lg_sess.SendPacket(buff, buff_len, pt, false, diff_nsec);
		checkerror (status);
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
					if (lg_callback) {
						lg_callback(pack->GetPayloadData(),
								pack->GetPayloadLength(), pack->GetPayloadType());
					}

					lg_sess.DeletePacket(pack);
				}
			} while (lg_sess.GotoNextSourceWithData());
		}

		lg_sess.EndDataAccess();

#ifndef RTP_SUPPORT_THREAD
		status = lg_sess.Poll();
		checkerror (status);
#endif // RTP_SUPPORT_THREAD
	}
	return NULL;
}

static bool is_init = false;
int init_rtp() {
	if (is_init) {
		return;
	}
	is_init = true;

	uint16_t portbase, destport;
	uint32_t destip;
	std::string ipstr;
	int status, i, num;

	portbase = 9004;
	destport = 9002;

	ipstr = "192.168.4.2";
	destip = inet_addr(ipstr.c_str());
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
		return;
	}
	lg_receive_run = false;
	pthread_join(lg_receive_thread, NULL);
	lg_sess.BYEDestroy(RTPTime(10, 0), 0, 0);
	is_init = false;
}
