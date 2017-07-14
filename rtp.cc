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
#include <arpa/inet.h>
#include <sys/time.h>

#include <iostream>
#include <string>
#include <list>

#include "rtp.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "mrevent.h"

#ifdef __cplusplus
}
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

//#define USE_SOCKET

#if defined USE_JRTP
#include "rtpsession.h"
#include "rtpudpv4transmitter.h"
#include "rtpipv4address.h"
#include "rtpsessionparams.h"
#include "rtperrors.h"
#include "rtplibraryversion.h"
#include "rtppacket.h"

using namespace jrtplib;

static RTPSession lg_sess;

#else

#if defined USE_SOCKET
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#endif

struct RTPHeader {
#ifdef RTP_BIG_ENDIAN
	uint8_t version:2;
	uint8_t padding:1;
	uint8_t extension:1;
	uint8_t csrccount:4;

	uint8_t marker:1;
	uint8_t payloadtype:7;
#else // little endian
	uint8_t csrccount :4;
	uint8_t extension :1;
	uint8_t padding :1;
	uint8_t version :2;

	uint8_t payloadtype :7;
	uint8_t marker :1;
#endif // RTP_BIG_ENDIAN

	uint16_t sequencenumber;
	uint32_t timestamp;
	uint32_t ssrc;
};

class RTPPacket {
public:
	RTPPacket(size_t packetlength) :
			packetlength(packetlength) {
		packet = new uint8_t[packetlength];
	}
	~RTPPacket() {
		if (packet) {
			delete packet;
			packet = NULL;
		}
	}
	uint16_t GetSequenceNumber() const {
		return seqnr;
	}
	uint32_t GetTimestamp() const {
		return timestamp;
	}
	uint8_t *GetPacketData() const {
		return packet;
	}
	uint8_t *GetPayloadData() const {
		return packet + sizeof(struct RTPHeader);
	}
	size_t GetPacketLength() const {
		return packetlength;
	}
	size_t GetPayloadLength() const {
		return packetlength - sizeof(struct RTPHeader);
	}
	uint8_t GetPayloadType() const {
		return payloadtype;
	}
	void LoadHeader() {
		struct RTPHeader *rtpheader_p = (struct RTPHeader*) packet;
		seqnr = ntohs(rtpheader_p->sequencenumber);
		payloadtype = rtpheader_p->payloadtype;
		timestamp = ntohl(rtpheader_p->timestamp);
	}
	void StoreHeader() {
		struct RTPHeader *rtpheader_p = (struct RTPHeader*) packet;
		rtpheader_p->sequencenumber = htons(seqnr);
		rtpheader_p->payloadtype = payloadtype;
		rtpheader_p->timestamp = htonl(timestamp);
	}
	uint8_t payloadtype;
	uint16_t seqnr;
	uint32_t timestamp;
	size_t packetlength;
	uint8_t *packet;
};
static uint16_t lg_seqnr = 0;
static uint32_t lg_timestamp = 0;
static int lg_tx_fd = -1;

static pthread_t lg_buffering_thread;
static MREVENT_T lg_buffering_ready;
static pthread_mutex_t lg_buffering_queue_mlock = PTHREAD_MUTEX_INITIALIZER;
static std::list<RTPPacket*> lg_buffering_queue;
#endif

static bool lg_receive_run = false;
static pthread_t lg_receive_thread;
static pthread_mutex_t lg_mlock = PTHREAD_MUTEX_INITIALIZER;

static char lg_record_path[256];
static int lg_record_fd = -1;
static pthread_t lg_record_thread;
static MREVENT_T lg_record_packet_ready;
static pthread_mutex_t lg_record_packet_queue_mlock = PTHREAD_MUTEX_INITIALIZER;
static std::list<RTPPacket*> lg_record_packet_queue;

static char lg_load_path[256];
static int lg_load_fd = -1;
static pthread_t lg_load_thread;
static bool lg_auto_play = false;
static bool lg_is_looping = false;
static MREVENT_T lg_play_time_updated;
static uint64_t lg_play_time = 0;

static RTP_CALLBACK lg_callback = NULL;

static float lg_bandwidth = 0;
static float lg_bandwidth_limit = 100 * 1024 * 1024; //100Mbps

void rtp_set_callback(RTP_CALLBACK callback) {
	lg_callback = callback;
}

void rtp_set_auto_play(bool value) {
	lg_auto_play = value;
}

void rtp_set_is_looping(bool value) {
	lg_is_looping = value;
}

#ifdef USE_JRTP
static void checkerror(int rtperr) {
	if (rtperr < 0) {
		std::cout << "ERROR: " << RTPGetErrorString(rtperr) << std::endl;
		exit(-1);
	}
}
#endif

float rtp_get_bandwidth() {
	return lg_bandwidth;
}

int rtp_sendpacket(unsigned char *data, int data_len, int pt) {
	pthread_mutex_lock(&lg_mlock);
	{
		static int last_data_len = 0;
		static struct timeval last_time = { };
		struct timeval time = { };
		gettimeofday(&time, NULL);
		struct timeval diff;
		timersub(&time, &last_time, &diff);
		int diff_usec = diff.tv_sec * 1000000 + (float) diff.tv_usec;
		if (diff_usec == 0) {
			diff_usec = 1;
		}
		{ //bandwidth
			float tmp = 8.0f * last_data_len / diff_usec; //Mbps
			float w = (float) diff_usec / 1000000 / 10;
			lg_bandwidth = lg_bandwidth * (1.0 - w) + tmp * w;
		}

#ifdef USE_JRTP
		int status = lg_sess.SendPacket(data, data_len, pt, false, diff_usec);
		checkerror(status);
#elif defined USE_SOCKET
#else
		if (lg_tx_fd < 0) {
			lg_tx_fd = open("rtp_tx", O_WRONLY);
		}
		if (lg_tx_fd >= 0) {
			unsigned char header[8];
			header[0] = 0xFF;
			header[1] = 0xE1;
			unsigned short len = sizeof(header) + sizeof(struct RTPHeader)
					+ data_len;
			for (int i = 0; i < 2; i++) {
				header[2 + i] = (len >> (8 * i)) & 0xFF;
			}
			header[4] = (unsigned char) 'r';
			header[5] = (unsigned char) 't';
			header[6] = (unsigned char) 'p';
			header[7] = (unsigned char) '\0';
			write(lg_tx_fd, header, sizeof(header));

			lg_seqnr++;
			lg_timestamp += diff_usec;

			RTPPacket *pack = new RTPPacket(sizeof(struct RTPHeader));
			pack->seqnr = lg_seqnr;
			pack->timestamp = lg_timestamp;
			pack->payloadtype = pt;
			pack->StoreHeader();
			write(lg_tx_fd, pack->GetPacketData(), pack->GetPacketLength());
			write(lg_tx_fd, data, data_len);
			delete pack;
		}
#endif
		if (lg_bandwidth_limit > 0) { //limit bandwidth
			struct timeval time2 = { };
			gettimeofday(&time2, NULL);
			struct timeval diff;
			timersub(&time2, &time, &diff);
			int diff_usec = diff.tv_sec * 1000000 + diff.tv_usec;
			float cal_usec = 8.0f * data_len / (lg_bandwidth_limit / 1000000);
			if (diff_usec < cal_usec) {
				int need_to_wait = MIN(cal_usec - diff_usec, 1000000);
				usleep(need_to_wait);
			}
		}

		last_time = time;
		last_data_len = data_len;
	}
	pthread_mutex_unlock(&lg_mlock);
	return 0;
}
#ifdef USE_JRTP
#else
static void *buffering_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "RTP BUFFERING");

	int rx_fd = open("rtp_rx", O_RDONLY);
	if (rx_fd < 0) {
		return NULL;
	}
	while (lg_receive_run) {
		int size = 4 * 1024;
		RTPPacket *raw_pack = new RTPPacket(size);
		raw_pack->packetlength = read(rx_fd, raw_pack->GetPacketData(),
				raw_pack->GetPacketLength());
		if (raw_pack->packetlength <= 0) {
			delete raw_pack;
			continue;
		}

		pthread_mutex_lock(&lg_buffering_queue_mlock);
		lg_buffering_queue.push_back(raw_pack);
		mrevent_trigger(&lg_buffering_ready);
		pthread_mutex_unlock(&lg_buffering_queue_mlock);
	}
	return NULL;
}
#endif

static void *receive_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "RTP RECEIVE");

#ifdef USE_JRTP
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
								pack->GetPayloadType(),
								pack->GetSequenceNumber());
					}
					if (lg_record_fd > 0) {
						pthread_mutex_lock(&lg_record_packet_queue_mlock);
						lg_record_packet_queue.push_back(pack);
						mrevent_trigger(&lg_record_packet_ready);
						pthread_mutex_unlock(&lg_record_packet_queue_mlock);
					} else {
						lg_sess.DeletePacket(pack);
					}
				}
			}while (lg_sess.GotoNextSourceWithData());
		}

		lg_sess.EndDataAccess();

#ifndef RTP_SUPPORT_THREAD
		status = lg_sess.Poll();
		checkerror(status);
#endif // RTP_SUPPORT_THREAD
	}
#else
	int marker = 0;
	int xmp_len = 0;
	int xmp_pos = 0;
	bool xmp = false;
	RTPPacket *pack = NULL;
	while (lg_receive_run) {
		int res = mrevent_wait(&lg_buffering_ready, 1000);
		if (res != 0) {
			continue;
		}

		RTPPacket *raw_pack = NULL;
		pthread_mutex_lock(&lg_buffering_queue_mlock);
		raw_pack = *(lg_buffering_queue.begin());
		lg_buffering_queue.pop_front();
		if (lg_buffering_queue.empty()) {
			mrevent_reset(&lg_buffering_ready);
		}
		pthread_mutex_unlock(&lg_buffering_queue_mlock);

		int data_len = raw_pack->GetPacketLength();
		unsigned char *buff = raw_pack->GetPacketData();

		for (int i = 0; i < data_len; i++) {
			if (xmp) {
				if (xmp_pos == 2) {
					xmp_len = buff[i];
					xmp_pos++;
				} else if (xmp_pos == 3) {
					xmp_len += buff[i] << 8;
					xmp_pos++;
				} else if (xmp_pos == 4) {
					if (buff[i] == 'r') {
						xmp_pos++;
					} else {
						xmp = false;
					}
				} else if (xmp_pos == 5) {
					if (buff[i] == 't') {
						xmp_pos++;
					} else {
						xmp = false;
					}
				} else if (xmp_pos == 6) {
					if (buff[i] == 'p') {
						xmp_pos++;
					} else {
						xmp = false;
					}
				} else if (xmp_pos == 7) { // rtp header
					if (buff[i] == '\0') {
						xmp_pos++;
					} else {
						xmp = false;
					}
				} else {
					if (xmp_pos == 8) {
						pack = new RTPPacket(xmp_len - 8);
					}
					if (i + (xmp_len - xmp_pos) <= data_len) {
						memcpy(pack->GetPacketData() + xmp_pos - 8, &buff[i],
								xmp_len - xmp_pos);
						i += xmp_len - xmp_pos - 1;
						xmp_pos = xmp_len;
						pack->LoadHeader();
						if (lg_callback && lg_load_fd < 0) {
							lg_callback(pack->GetPayloadData(),
									pack->GetPayloadLength(),
									pack->GetPayloadType(),
									pack->GetSequenceNumber());
						}
						if (lg_record_fd > 0) {
							pthread_mutex_lock(&lg_record_packet_queue_mlock);
							lg_record_packet_queue.push_back(pack);
							mrevent_trigger(&lg_record_packet_ready);
							pthread_mutex_unlock(&lg_record_packet_queue_mlock);
						} else {
							delete pack;
						}
						pack = NULL;
						xmp = false;
					} else {
						int rest_in_buff = data_len - i;
						memcpy(pack->GetPacketData() + xmp_pos - 8, &buff[i],
								rest_in_buff);
						i = data_len - 1;
						xmp_pos += rest_in_buff;
					}
				}
			} else {
				if (marker) {
					marker = 0;
					if (buff[i] == 0xE1) { //xmp
						xmp = true;
						xmp_pos = 2;
						xmp_len = 0;
					}
				} else if (buff[i] == 0xFF) {
					marker = 1;
				}
			}
		}
		delete raw_pack;
	}
#endif
	return NULL;
}

static void *record_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "RTP RECORD");

	uint64_t num_of_bytes = 0;
	uint64_t last_sync_bytes = 0;
	const uint64_t MB = 1024 * 1024; // 64MB
	const uint64_t SYNC_THRESHOLD = 64 * MB; // 64MB
	RTPPacket *pack;
	while (lg_record_fd >= 0) {
		int res = mrevent_wait(&lg_record_packet_ready, 1000);
		if (res != 0) {
			continue;
		}
		int fd = lg_record_fd;
		if (fd < 0) { //for thread safe
			continue;
		}

		pthread_mutex_lock(&lg_record_packet_queue_mlock);
		pack = *(lg_record_packet_queue.begin());
		lg_record_packet_queue.pop_front();
		if (lg_record_packet_queue.empty()) {
			mrevent_reset(&lg_record_packet_ready);
		}
		pthread_mutex_unlock(&lg_record_packet_queue_mlock);

		unsigned char header[8];
		header[0] = 0xFF;
		header[1] = 0xE1;
		unsigned short len = pack->GetPacketLength() + sizeof(header);
		for (int i = 0; i < 2; i++) {
			header[2 + i] = (len >> (8 * i)) & 0xFF;
		}
		header[4] = (unsigned char) 'r';
		header[5] = (unsigned char) 't';
		header[6] = (unsigned char) 'p';
		header[7] = (unsigned char) '\0';
		write(fd, header, sizeof(header));
		write(fd, pack->GetPacketData(), pack->GetPacketLength());

		num_of_bytes += len;
		if (num_of_bytes - last_sync_bytes > SYNC_THRESHOLD) {
			printf("fsync %lluMB\n", num_of_bytes / MB);
			last_sync_bytes = num_of_bytes;
			fsync(fd); //this avoid that file size would be zero after os crash
		}
#ifdef USE_JRTP
		lg_sess.DeletePacket(pack);
#else
		delete pack;
#endif
	}
	pthread_mutex_lock(&lg_record_packet_queue_mlock);
	while (!lg_record_packet_queue.empty()) {
		pack = *(lg_record_packet_queue.begin());
		lg_record_packet_queue.pop_front();
#ifdef USE_JRTP
		lg_sess.DeletePacket(pack);
#else
		delete pack;
#endif
	}
	mrevent_reset(&lg_record_packet_ready);
	pthread_mutex_unlock(&lg_record_packet_queue_mlock);
	return NULL;
}

static void *load_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "RTP LOAD");

	void **args = (void**) arg;
	RTP_LOADING_CALLBACK callback = (RTP_LOADING_CALLBACK) args[0];
	void *user_data = args[1];
	free(arg);

	unsigned char buff[RTP_MAXPAYLOADSIZE + sizeof(struct RTPHeader)];
	unsigned int last_timestamp = 0;
	uint64_t current_play_time = 0;
	struct timeval last_time = { };
	bool is_first = true;
	int ret = 0;
	lg_play_time = 0;
	while (lg_load_fd >= 0) {
		int raw_header_size = 8;
		int read_len;
		struct RTPHeader *header = (struct RTPHeader *) buff;
		read_len = read(lg_load_fd, buff, raw_header_size);
		if (read_len == 0) { //eof
			if (lg_is_looping) {
				lseek(lg_load_fd, 0, SEEK_SET);
				continue;
			} else {
				break;
			}
		}
		if (read_len != 8 || buff[0] != 0xFF || buff[1] != 0xE1
				|| buff[4] != 'r' || buff[5] != 't' || buff[6] != 'p'
				|| buff[7] != '\0') {
			//error
			ret = -1;
			perror("rtp error\n");
			break;
		}
		unsigned short len = 0;
		for (int i = 0; i < 2; i++) {
			len += (buff[i + 2] << (8 * i));
		}
		len -= raw_header_size;
		if (len > sizeof(buff)) {
			//error
			ret = -1;
			break;
		}
		read_len = read(lg_load_fd, buff, len);
		if (read_len != len) {
			//error
			ret = -1;
			break;
		}

		{ //wait
			unsigned int timestamp = ntohl(header->timestamp);
			struct timeval time = { };
			gettimeofday(&time, NULL);
			if (is_first) {
				is_first = false;
				last_timestamp = timestamp;
				gettimeofday(&last_time, NULL);
			}
			int elapsed_usec;
			if (timestamp < last_timestamp) {
				elapsed_usec = timestamp + (UINT_MAX - last_timestamp);
			} else {
				elapsed_usec = timestamp - last_timestamp;
			}
			current_play_time += elapsed_usec;
			if (lg_auto_play) {
				struct timeval diff;
				timersub(&time, &last_time, &diff);
				int diff_usec = diff.tv_sec * 1000000 + (float) diff.tv_usec;

				if (diff_usec < elapsed_usec) {
					usleep(MIN(elapsed_usec - diff_usec, 1000000));
				}
				last_time = time;
				lg_play_time = current_play_time;
			}
			while (lg_load_fd >= 0 && !lg_auto_play) {
				if (current_play_time <= lg_play_time) {
					break;
				} else {
					mrevent_reset(&lg_play_time_updated);
				}
				mrevent_wait(&lg_play_time_updated, 1000);
			}
			last_timestamp = timestamp;
		}

		if (lg_callback) {
			lg_callback(buff + sizeof(struct RTPHeader),
					len - sizeof(struct RTPHeader), header->payloadtype,
					ntohs(header->sequencenumber));
		}
	}
	if (callback) {
		callback(user_data, ret);
	}
	return NULL;
}

static bool is_init = false;
int init_rtp(unsigned short portbase, char *destip_str, unsigned short destport,
		float bandwidth_limit) {
	if (is_init) {
		return -1;
	}
	is_init = true;

	lg_bandwidth_limit = bandwidth_limit;
	lg_receive_run = true;

#ifdef USE_JRTP
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

	sessparams.SetOwnTimestampUnit(1.0 / 1E6);//micro sec
	sessparams.SetMaximumPacketSize(
			RTP_MAXPAYLOADSIZE + sizeof(struct RTPHeader));
	sessparams.SetAcceptOwnPackets(true);
	transparams.SetPortbase(portbase);

	status = lg_sess.Create(sessparams, &transparams);
	checkerror(status);

	RTPIPv4Address addr(destip, destport);

	status = lg_sess.AddDestination(addr);
	checkerror(status);
#else
	pthread_create(&lg_buffering_thread, NULL, buffering_thread_func,
			(void*) NULL);
	mrevent_init(&lg_buffering_ready);
#endif

	pthread_create(&lg_receive_thread, NULL, receive_thread_func, (void*) NULL);

	mrevent_init(&lg_record_packet_ready);
	mrevent_init(&lg_play_time_updated);

	return 0;
}

int deinit_rtp() {
	if (!is_init) {
		return -1;
	}
	lg_receive_run = false;
	pthread_join(lg_receive_thread, NULL);
#ifdef USE_JRTP
	lg_sess.BYEDestroy(RTPTime(10, 0), 0, 0);
#else
	if (lg_tx_fd >= 0) {
		close(lg_tx_fd);
		lg_tx_fd = -1;
	}
	pthread_join(lg_buffering_thread, NULL);
#endif
	is_init = false;
	return 0;
}

void rtp_start_recording(char *path) {
	rtp_stop_recording();
	strcpy(lg_record_path, path);
	lg_record_fd = open(lg_record_path, O_CREAT | O_WRONLY | O_TRUNC);
	pthread_create(&lg_record_thread, NULL, record_thread_func, (void*) NULL);
}

void rtp_stop_recording() {
	if (lg_record_fd > 0) {
		int fd = lg_record_fd;
		lg_record_fd = -1;
		pthread_join(lg_record_thread, NULL);
		close(fd);
	}
}

bool rtp_is_recording(char **path) {
	if (path) {
		*path = lg_record_path;
	}
	return (lg_record_fd > 0);
}

bool rtp_start_loading(char *path, bool auto_play, bool is_looping,
		RTP_LOADING_CALLBACK callback, void *user_data) {
	rtp_stop_loading();
	strcpy(lg_load_path, path);
	lg_auto_play = auto_play;
	lg_is_looping = is_looping;
	lg_load_fd = open(lg_load_path, O_RDONLY);
	if (lg_load_fd > 0) {
		void **args = (void**) malloc(sizeof(void*) * 2);
		args[0] = (void*) callback;
		args[1] = user_data;
		pthread_create(&lg_load_thread, NULL, load_thread_func, (void*) args);
		return true;
	} else {
		printf("failed to open %s\n", lg_load_path);
		return false;
	}
}

void rtp_increment_loading(int elapsed_usec) {
	lg_play_time += elapsed_usec;
	mrevent_trigger(&lg_play_time_updated);
}

void rtp_stop_loading() {
	if (lg_load_fd > 0) {
		int fd = lg_load_fd;
		lg_load_fd = -1;
		pthread_join(lg_load_thread, NULL);
		close(fd);
	}
}

bool rtp_is_loading(char **path) {
	if (path) {
		*path = lg_load_path;
	}
	return (lg_load_fd > 0);
}
