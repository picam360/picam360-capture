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

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <signal.h>

#ifdef _WIN64
//define something for Windows (64-bit)
#elif _WIN32
//define something for Windows (32-bit)
#elif __APPLE__
#include "TargetConditionals.h"
#if TARGET_OS_IPHONE && TARGET_IPHONE_SIMULATOR
// define something for simulator
#elif TARGET_OS_IPHONE
// define something for iphone
#else
#define TARGET_OS_OSX 1
// define something for OSX
#define pthread_setname_np(a, b) pthread_setname_np(b)
#endif
#elif __linux
// linux
#elif __unix // all unices not caught above
// Unix
#elif __posix
// POSIX
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

struct RTP_CALLBACK_PAIR {
	RTP_CALLBACK callback;
	void *user_data;
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
		ssrc = ntohl(rtpheader_p->ssrc);
	}
	void StoreHeader() {
		struct RTPHeader *rtpheader_p = (struct RTPHeader*) packet;
		rtpheader_p->sequencenumber = htons(seqnr);
		rtpheader_p->payloadtype = payloadtype;
		rtpheader_p->timestamp = htonl(timestamp);
		rtpheader_p->ssrc = htonl(ssrc);
	}
	uint8_t payloadtype;
	uint16_t seqnr;
	uint32_t timestamp;
	uint32_t ssrc;
	size_t packetlength;
	uint8_t *packet;
};

typedef struct _RTP_T {
	bool is_init = false;
	uint16_t seqnr = 0;
	int tx_fd = -1;

	bool buffering_run = false;
	pthread_t buffering_thread;
	MREVENT_T buffering_ready;
	pthread_mutex_t buffering_queue_mlock = PTHREAD_MUTEX_INITIALIZER;
	std::list<RTPPacket*> buffering_queue;

	bool receive_run = false;
	pthread_t receive_thread;
	pthread_mutex_t mlock = PTHREAD_MUTEX_INITIALIZER;

	char record_path[256];
	int record_fd = -1;
	pthread_t record_thread;
	MREVENT_T record_packet_ready;
	pthread_mutex_t record_packet_queue_mlock = PTHREAD_MUTEX_INITIALIZER;
	std::list<RTPPacket*> record_packet_queue;

	char load_path[256];
	int load_fd = -1;
	pthread_t load_thread;
	bool auto_play = false;
	bool is_looping = false;
	MREVENT_T play_time_updated;
	uint64_t play_time = 0;
	float play_speed = 1.0;

	pthread_mutex_t callbacks_mlock = PTHREAD_MUTEX_INITIALIZER;
	std::list<struct RTP_CALLBACK_PAIR> callbacks;

	float bandwidth = 0;
	float bandwidth_limit = 100 * 1024 * 1024; //100Mbps

	char *tx_buffer = NULL;
	int tx_buffer_cur = 0;
	sockaddr_in tx_addr = { };
	sockaddr_in rx_addr = { };
	enum RTP_SOCKET_TYPE rx_socket_type = RTP_SOCKET_TYPE_NONE;
	enum RTP_SOCKET_TYPE tx_socket_type = RTP_SOCKET_TYPE_NONE;
	int rx_buffer_size = RTP_MAXPACKETSIZE;
	int tx_buffer_size = RTP_MAXPACKETSIZE;
} RTP_T;

int send_via_socket(RTP_T *_this, int fd, const unsigned char *data, int data_len, bool flush) {
	for (int i = 0; i < data_len;) {
		int buffer_space = _this->tx_buffer_size - _this->tx_buffer_cur;
		if (buffer_space < (data_len - i)) {
			memcpy(_this->tx_buffer + _this->tx_buffer_cur, data + i, buffer_space);
			_this->tx_buffer_cur += buffer_space;
			i += buffer_space;

			int size = _this->tx_buffer_cur;
			_this->tx_buffer_cur = 0;

			int actsize = sendto(fd, _this->tx_buffer, size, 0, (struct sockaddr *) &_this->tx_addr, sizeof(_this->tx_addr));
			if (actsize != size) {
				perror("sendto() failed.");
				return -1;
			}
		} else {
			memcpy(_this->tx_buffer + _this->tx_buffer_cur, data + i, data_len - i);
			_this->tx_buffer_cur += data_len - i;
			i = data_len;
		}
	}
	if (flush && _this->tx_buffer_cur > 0) {
		int size = _this->tx_buffer_cur;
		_this->tx_buffer_cur = 0;

		int actsize = sendto(fd, _this->tx_buffer, size, 0, (struct sockaddr *) &_this->tx_addr, sizeof(_this->tx_addr));
		if (actsize != size) {
			perror("sendto() failed.");
			return -1;
		}
	}
	return data_len;
}

void rtp_add_callback(RTP_T *_this, RTP_CALLBACK callback, void *user_data) {
	struct RTP_CALLBACK_PAIR pair;
	pair.callback = callback;
	pair.user_data = user_data;

	pthread_mutex_lock(&_this->callbacks_mlock);
	_this->callbacks.push_back(pair);
	pthread_mutex_unlock(&_this->callbacks_mlock);
}

void rtp_set_auto_play(RTP_T *_this, bool value) {
	_this->auto_play = value;
}

void rtp_set_is_looping(RTP_T *_this, bool value) {
	_this->is_looping = value;
}

float rtp_get_bandwidth(RTP_T *_this) {
	return _this->bandwidth;
}

static int connect_timeout(struct sockaddr_in *client, int timeout) {
	int sock;
	int retval;
	fd_set set;
	struct timeval tv;

	/*** make socket discriptor ***/
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		close(sock);
		return -1;
	}
	/*** end make socket discriptor ***/

	/*** set socket nonblocking ***/
	retval = fcntl(sock, F_SETFL, O_NONBLOCK);
	if (retval < 0) {
		close(sock);
		return -1;
	}
	/*** end set socket nonblocking ***/

	/*** connect time out***/
	retval = connect(sock, (struct sockaddr *) client, sizeof(*client));
	if (retval < 0) {
		if (errno == EINPROGRESS) {
			tv.tv_sec = timeout;
			tv.tv_usec = 0;
			FD_ZERO(&set);
			FD_SET(sock, &set);
			retval = select(sock + 1, NULL, &set, NULL, &tv);
			if (retval < 0) {
				close(sock);
				return -1;
			} else if (retval > 0) {
				/*connect*/
				retval = connect(sock, (struct sockaddr *) client, sizeof(*client));
				if (retval < 0) {
					close(sock);
					return -1;
				}
			} else {
				/** Time out **/
				close(sock);
				return -1;
			}
		} else {
			close(sock);
			return -1;
		}
	}
	/*** end connect time out ***/

	/*** set socket blocking(option) ***/
	retval = fcntl(sock, F_SETFL, 0);
	if (retval < 0) {
		close(sock);
		return -1;
	}
	/*** end set socket blocking(option) ***/

	/*** end close socket discriptor ***/
	return sock;
}

int rtp_sendpacket(RTP_T *_this, const unsigned char *data, int data_len, int pt) {
	pthread_mutex_lock(&_this->mlock);
	{
		int ret;
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
			_this->bandwidth = _this->bandwidth * (1.0 - w) + tmp * w;
		}

		if (_this->tx_fd < 0) {
			switch (_this->tx_socket_type) {
			case RTP_SOCKET_TYPE_TCP:
				_this->tx_fd = connect_timeout(&_this->tx_addr, 5);
				break;
			case RTP_SOCKET_TYPE_UDP:
				_this->tx_fd = socket(AF_INET, SOCK_DGRAM, 0);
				break;
			case RTP_SOCKET_TYPE_FIFO:
				_this->tx_fd = open("rtp_tx", O_WRONLY);
				break;
			default:
				break;
			}
		}
		if (_this->tx_fd >= 0) {

			_this->seqnr++;

			RTPPacket *pack = new RTPPacket(sizeof(RTPHeader));
			pack->seqnr = _this->seqnr;
			pack->timestamp = time.tv_sec;
			pack->ssrc = time.tv_usec;
			pack->payloadtype = pt;
			pack->StoreHeader();

			unsigned char header[8];
			unsigned short len = sizeof(header) + sizeof(struct RTPHeader) + data_len;
			header[0] = 0xFF;
			header[1] = 0xE1;
			for (int i = 0; i < 2; i++) {
				header[2 + i] = (len >> (8 * i)) & 0xFF;
			}
			header[4] = (unsigned char) 'r';
			header[5] = (unsigned char) 't';
			header[6] = (unsigned char) 'p';
			header[7] = (unsigned char) '\0';

			switch (_this->tx_socket_type) {
			case RTP_SOCKET_TYPE_TCP:
			case RTP_SOCKET_TYPE_UDP:
				if (1) {
					uint8_t *data_ary[3] = { header, pack->GetPacketData(), (uint8_t*) data };
					int datalen_ary[3] = { sizeof(header), (int) pack->GetPacketLength(), data_len };
					bool flush_ary[3] = { false, false, false };
					for (int i = 0; i < 3; i++) {
						int size = send_via_socket(_this, _this->tx_fd, data_ary[i], datalen_ary[i], flush_ary[i]);
						if (size < 0) {
							close(_this->tx_fd);
							_this->tx_fd = -1;
							break;
						}
					}
				}
				break;
			case RTP_SOCKET_TYPE_FIFO:
				ret = write(_this->tx_fd, header, sizeof(header));
				ret = write(_this->tx_fd, pack->GetPacketData(), pack->GetPacketLength());
				ret = write(_this->tx_fd, data, data_len);
				break;
			default:
				break;
			}
			delete pack;
		}
		if (_this->bandwidth_limit > 0) { //limit bandwidth
			struct timeval time2 = { };
			gettimeofday(&time2, NULL);
			struct timeval diff;
			timersub(&time2, &time, &diff);
			int diff_usec = diff.tv_sec * 1000000 + diff.tv_usec;
			float cal_usec = 8.0f * data_len / (_this->bandwidth_limit / 1000000);
			if (diff_usec < cal_usec) {
				int need_to_wait = MIN(cal_usec - diff_usec, 1000000);
				usleep(need_to_wait);
			}
		}

		last_time = time;
		last_data_len = data_len;
	}
	pthread_mutex_unlock(&_this->mlock);
	return 0;
}
void rtp_flush(RTP_T *_this) {
	pthread_mutex_lock(&_this->mlock);
	send_via_socket(_this, _this->tx_fd, NULL, 0, true);
	pthread_mutex_unlock(&_this->mlock);
}

static void *buffering_thread_func(void* arg) {
	RTP_T *_this = (RTP_T*) arg;
	pthread_setname_np(pthread_self(), "RTP BUFFERING");

	int srv_fd = -1;
	while (_this->buffering_run) {
		int rx_fd = -1;
		switch (_this->rx_socket_type) {
		case RTP_SOCKET_TYPE_TCP:
			if (1) {
				sockaddr_in client_addr = { };
				socklen_t client_addr_size = sizeof(client_addr);

				if (srv_fd < 0) {
					srv_fd = socket(AF_INET, SOCK_STREAM, 0);
					int status = bind(srv_fd, (struct sockaddr*) &_this->rx_addr, sizeof(_this->rx_addr));
					if (status < 0) {
						perror("bind() failed.");
					}
				}
				listen(srv_fd, 1);
				printf("Waiting for connection ...\n");
				rx_fd = accept(srv_fd, (struct sockaddr *) &client_addr, &client_addr_size);
				printf("Connected from %s\n", inet_ntoa(client_addr.sin_addr));
			}
			break;
		case RTP_SOCKET_TYPE_UDP:
			if (1) {
				rx_fd = socket(AF_INET, SOCK_DGRAM, 0);
				int status = bind(rx_fd, (struct sockaddr*) &_this->rx_addr, sizeof(_this->rx_addr));
				if (status < 0) {
					perror("bind() failed.");
				}
			}
			break;
		case RTP_SOCKET_TYPE_FIFO:
			rx_fd = open("rtp_rx", O_RDONLY);
			break;
		default:
			break;
		}
		if (rx_fd < 0) {
			return NULL;
		}
		while (_this->buffering_run) {
			RTPPacket *raw_pack = NULL;
			switch (_this->rx_socket_type) {
			case RTP_SOCKET_TYPE_TCP:
			case RTP_SOCKET_TYPE_UDP:
				if (1) {
					raw_pack = new RTPPacket(_this->rx_buffer_size);
					raw_pack->packetlength = recvfrom(rx_fd, raw_pack->GetPacketData(), raw_pack->GetPacketLength(), 0, NULL, NULL);
				}
				break;
			case RTP_SOCKET_TYPE_FIFO:
				if (1) {
					raw_pack = new RTPPacket(_this->rx_buffer_size);
					raw_pack->packetlength = read(rx_fd, raw_pack->GetPacketData(), raw_pack->GetPacketLength());
				}
				break;
			default:
				break;
			}

			if (raw_pack == NULL) {
				continue;
			} else if (raw_pack->packetlength <= 0) {
				delete raw_pack;
				if (errno == EAGAIN) {
					continue;
				} else {
					close(rx_fd);
					rx_fd = -1;
					printf("connection closed\n");
					break;
				}
			}

			if (_this->load_fd < 0) {
				pthread_mutex_lock(&_this->buffering_queue_mlock);
				_this->buffering_queue.push_back(raw_pack);
				mrevent_trigger(&_this->buffering_ready);
				pthread_mutex_unlock(&_this->buffering_queue_mlock);
			}
		}
	}
	return NULL;
}

static void *receive_thread_func(void* arg) {
	RTP_T *_this = (RTP_T*) arg;
	pthread_setname_np(pthread_self(), "RTP RECEIVE");

	//for loading
	unsigned int last_timestamp = 0;
	uint64_t current_play_time = 0;
	struct timeval last_time = { };
	bool is_first = true;

	int marker = 0;
	int xmp_len = 0;
	int xmp_pos = 0;
	bool xmp = false;
	RTPPacket *pack = NULL;
	while (_this->receive_run) {
		int res = mrevent_wait(&_this->buffering_ready, 100000);
		if (res != 0) {
			continue;
		}

		RTPPacket *raw_pack = NULL;
		pthread_mutex_lock(&_this->buffering_queue_mlock);
		raw_pack = *(_this->buffering_queue.begin());
		_this->buffering_queue.pop_front();
		if (_this->buffering_queue.empty()) {
			mrevent_reset(&_this->buffering_ready);
		}
		pthread_mutex_unlock(&_this->buffering_queue_mlock);

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
						memcpy(pack->GetPacketData() + xmp_pos - 8, &buff[i], xmp_len - xmp_pos);
						i += xmp_len - xmp_pos - 1;
						xmp_pos = xmp_len;
						pack->LoadHeader();
						if (_this->load_fd >= 0) { //wait
							struct timeval time = { };
							gettimeofday(&time, NULL);
							if (is_first) {
								is_first = false;
								last_timestamp = pack->timestamp;
								gettimeofday(&last_time, NULL);
							}
							int elapsed_usec;
							if (pack->timestamp < last_timestamp) {
								elapsed_usec = pack->timestamp + (UINT_MAX - last_timestamp);
							} else {
								elapsed_usec = pack->timestamp - last_timestamp;
							}
							current_play_time += elapsed_usec;
							if (_this->auto_play) {
								struct timeval diff;
								timersub(&time, &last_time, &diff);
								float diff_usec = diff.tv_sec * 1000000 + (float) diff.tv_usec;
								float _elapsed_usec = ((float) elapsed_usec / _this->play_speed);

								if (diff_usec < _elapsed_usec) {
									usleep(MIN(_elapsed_usec - diff_usec, 1000000));
								}
								last_time = time;
								_this->play_time = current_play_time;
							}
							while (_this->load_fd >= 0 && !_this->auto_play) {
								if (current_play_time <= _this->play_time) {
									break;
								} else {
									mrevent_reset(&_this->play_time_updated);
								}
								mrevent_wait(&_this->play_time_updated, 100000);
							}
							last_timestamp = pack->timestamp;
						}

						pthread_mutex_lock(&_this->callbacks_mlock);
						for (std::list<struct RTP_CALLBACK_PAIR>::iterator it = _this->callbacks.begin(); it != _this->callbacks.end(); it++) {
							(*it).callback(pack->GetPayloadData(), pack->GetPayloadLength(), pack->GetPayloadType(), pack->GetSequenceNumber(), (*it).user_data);
						}
						pthread_mutex_unlock(&_this->callbacks_mlock);

						if (_this->record_fd > 0) {
							pthread_mutex_lock(&_this->record_packet_queue_mlock);
							_this->record_packet_queue.push_back(pack);
							mrevent_trigger(&_this->record_packet_ready);
							pthread_mutex_unlock(&_this->record_packet_queue_mlock);
						} else {
							delete pack;
						}
						pack = NULL;
						xmp = false;
					} else {
						int rest_in_buff = data_len - i;
						memcpy(pack->GetPacketData() + xmp_pos - 8, &buff[i], rest_in_buff);
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
	return NULL;
}

static void *record_thread_func(void* arg) {
	RTP_T *_this = (RTP_T*) arg;
	pthread_setname_np(pthread_self(), "RTP RECORD");

	int ret;
	uint64_t num_of_bytes = 0;
	uint64_t last_sync_bytes = 0;
	const uint64_t MB = 1024 * 1024; // 64MB
	const uint64_t SYNC_THRESHOLD = 64 * MB; // 64MB
	RTPPacket *pack;
	while (_this->record_fd >= 0) {
		int res = mrevent_wait(&_this->record_packet_ready, 100000);
		if (res != 0) {
			continue;
		}
		int fd = _this->record_fd;
		if (fd < 0) { //for thread safe
			continue;
		}

		pthread_mutex_lock(&_this->record_packet_queue_mlock);
		pack = *(_this->record_packet_queue.begin());
		_this->record_packet_queue.pop_front();
		if (_this->record_packet_queue.empty()) {
			mrevent_reset(&_this->record_packet_ready);
		}
		pthread_mutex_unlock(&_this->record_packet_queue_mlock);

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
		ret = write(fd, header, sizeof(header));
		ret = write(fd, pack->GetPacketData(), pack->GetPacketLength());

		num_of_bytes += len;
		if (num_of_bytes - last_sync_bytes > SYNC_THRESHOLD) {
			printf("fsync %luMB\n", num_of_bytes / MB);
			last_sync_bytes = num_of_bytes;
			fsync(fd); //this avoid that file size would be zero after os crash
		}
		delete pack;
	}
	pthread_mutex_lock(&_this->record_packet_queue_mlock);
	while (!_this->record_packet_queue.empty()) {
		pack = *(_this->record_packet_queue.begin());
		_this->record_packet_queue.pop_front();
		delete pack;
	}
	mrevent_reset(&_this->record_packet_ready);
	pthread_mutex_unlock(&_this->record_packet_queue_mlock);
	return NULL;
}

static void *load_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "RTP LOAD");

	void **args = (void**) arg;
	RTP_T *_this = (RTP_T*) args[0];
	RTP_LOADING_CALLBACK callback = (RTP_LOADING_CALLBACK) args[1];
	void *user_data = args[2];
	free(arg);

	uint64_t num_of_bytes = 0;
	uint64_t last_sync_bytes = 0;
	const uint64_t MB = 1024 * 1024; // 64MB
	const uint64_t SYNC_THRESHOLD = 64 * MB; // 64MB
	int ret = 0;
	_this->play_time = 0;
	while (_this->load_fd >= 0) {
		{ //check if buffering enough
			bool needToWait = false;
			pthread_mutex_lock(&_this->buffering_queue_mlock);
			if (_this->buffering_queue.size() > 10) {
				needToWait = true;
			}
			pthread_mutex_unlock(&_this->buffering_queue_mlock);
			if (needToWait) {
				usleep(10 * 1000); //10ms
			}
		}
		RTPPacket *raw_pack = new RTPPacket(_this->rx_buffer_size);
		raw_pack->packetlength = read(_this->load_fd, raw_pack->GetPacketData(), raw_pack->GetPacketLength());

		if (raw_pack->packetlength <= 0) { //eof
			if (_this->is_looping) {
				num_of_bytes = 0;
				last_sync_bytes = 0;

				lseek(_this->load_fd, 0, SEEK_SET);
				delete raw_pack;
				continue;
			} else {
				break;
			}
		}
		pthread_mutex_lock(&_this->buffering_queue_mlock);
		_this->buffering_queue.push_back(raw_pack);
		mrevent_trigger(&_this->buffering_ready);
		pthread_mutex_unlock(&_this->buffering_queue_mlock);

		num_of_bytes += raw_pack->packetlength;
		if (num_of_bytes - last_sync_bytes > SYNC_THRESHOLD) {
			printf("loading info %luMB\n", num_of_bytes / MB);
			last_sync_bytes = num_of_bytes;
		}
	}
	if (callback) {
		callback(user_data, ret);
	}
	return NULL;
}

RTP_T *create_rtp(unsigned short portbase, enum RTP_SOCKET_TYPE rx_socket_type, char *destip_str, unsigned short destport, enum RTP_SOCKET_TYPE tx_socket_type, float bandwidth_limit) {
	RTP_T *_this = new RTP_T;

	rtp_set_buffer_size(_this, _this->rx_buffer_size, _this->tx_buffer_size);

	_this->bandwidth_limit = bandwidth_limit;

	signal(SIGPIPE, SIG_IGN);

	if(portbase != 0) {
		_this->rx_socket_type = rx_socket_type;

		_this->rx_addr.sin_family = AF_INET;
		_this->rx_addr.sin_port = htons(portbase);
		_this->rx_addr.sin_addr.s_addr = htonl(INADDR_ANY);

		_this->receive_run = true;
		pthread_create(&_this->receive_thread, NULL, receive_thread_func, (void*) _this);
	}

	if(destport != 0){
		_this->tx_socket_type = tx_socket_type;

		_this->tx_addr.sin_family = AF_INET;
		_this->tx_addr.sin_port = htons(destport);
		_this->tx_addr.sin_addr.s_addr = inet_addr(destip_str);

		_this->buffering_run = true;
		pthread_create(&_this->buffering_thread, NULL, buffering_thread_func, (void*) _this);
		mrevent_init(&_this->buffering_ready);
	}

	mrevent_init(&_this->record_packet_ready);
	mrevent_init(&_this->play_time_updated);

	return _this;
}

int rtp_set_buffer_size(RTP_T *_this, int rx_buffer_size, int tx_buffer_size) {
	_this->rx_buffer_size = rx_buffer_size;
	_this->tx_buffer_size = tx_buffer_size;
	if (_this->tx_buffer) {
		free(_this->tx_buffer);
		_this->tx_buffer = NULL;
	}
	_this->tx_buffer = (char*) malloc(_this->tx_buffer_size);
	return 0;
}

int delete_rtp(RTP_T **_this_p) {
	RTP_T *_this = *_this_p;

	_this->receive_run = false;
	pthread_join(_this->receive_thread, NULL);
	if (_this->tx_fd >= 0) {
		close(_this->tx_fd);
		_this->tx_fd = -1;
	}
	pthread_join(_this->buffering_thread, NULL);

	delete _this;
	*_this_p = NULL;

	return 0;
}

void rtp_start_recording(RTP_T *_this, char *path) {
	rtp_stop_recording(_this);
	strcpy(_this->record_path, path);
	_this->record_fd = open(_this->record_path, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
	pthread_create(&_this->record_thread, NULL, record_thread_func, (void*) _this);
}

void rtp_stop_recording(RTP_T *_this) {
	if (_this->record_fd > 0) {
		int fd = _this->record_fd;
		_this->record_fd = -1;
		pthread_join(_this->record_thread, NULL);
		close(fd);
	}
}

bool rtp_is_recording(RTP_T *_this, char **path) {
	if (path) {
		*path = _this->record_path;
	}
	return (_this->record_fd > 0);
}

bool rtp_start_loading(RTP_T *_this, char *path, bool auto_play, bool is_looping, RTP_LOADING_CALLBACK callback, void *user_data) {
	rtp_stop_loading(_this);
	strcpy(_this->load_path, path);
	_this->auto_play = auto_play;
	_this->is_looping = is_looping;
	_this->load_fd = open(_this->load_path, O_RDONLY);
	if (_this->load_fd > 0) {
		void **args = (void**) malloc(sizeof(void*) * 2);
		args[0] = (void*) _this;
		args[1] = (void*) callback;
		args[2] = user_data;
		pthread_create(&_this->load_thread, NULL, load_thread_func, (void*) args);
		return true;
	} else {
		printf("failed to open %s\n", _this->load_path);
		return false;
	}
}

void rtp_increment_loading(RTP_T *_this, int elapsed_usec) {
	_this->play_time += elapsed_usec;
	mrevent_trigger(&_this->play_time_updated);
}

void rtp_set_play_speed(RTP_T *_this, float play_speed) {
	_this->play_speed = play_speed;
}

void rtp_stop_loading(RTP_T *_this) {
	if (_this->load_fd > 0) {
		int fd = _this->load_fd;
		_this->load_fd = -1;
		pthread_join(_this->load_thread, NULL);
		close(fd);
	}
}

bool rtp_is_loading(RTP_T *_this, char **path) {
	if (path) {
		*path = _this->load_path;
	}
	return (_this->load_fd > 0);
}

const char *rtp_get_rtp_socket_type_str(enum RTP_SOCKET_TYPE type) {
	switch (type) {
	case RTP_SOCKET_TYPE_TCP:
		return "tcp";
	case RTP_SOCKET_TYPE_UDP:
		return "udp";
	case RTP_SOCKET_TYPE_FIFO:
		return "fifo";
	default:
		return "none";
	}
}

enum RTP_SOCKET_TYPE rtp_get_rtp_socket_type(const char *type_str) {
	if (type_str == NULL) {
		return RTP_SOCKET_TYPE_NONE;
	}
	switch (type_str[0]) {
	case 't':
		return RTP_SOCKET_TYPE_TCP;
	case 'u':
		return RTP_SOCKET_TYPE_UDP;
	case 'f':
		return RTP_SOCKET_TYPE_FIFO;
	default:
		return RTP_SOCKET_TYPE_NONE;
	}
}
