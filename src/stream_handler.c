
static void stream_callback(unsigned char *data, unsigned int data_len, void *frame_data, void *user_data) {
	FRAME_T *frame = (FRAME_T*) user_data;
	FRAME_INFO_T *frame_info = (FRAME_INFO_T*) frame_data;
	NALU_STREAM_DEF def = {};
	if (frame->output_mode == OUTPUT_MODE_STREAM) {
		switch (frame->output_type){
		case OUTPUT_TYPE_I420:
			{
				NALU_STREAM_DEF _def = {
						.SC = { 0x00, 0x00, 0x00, 0x01 },
						.SOI = { 0x49, 0x34 }, //'I', '4'
						.EOI = { 0x32, 0x30 }, //'2', '0'
						.SEI_CODE = 40 << 1,
				};
				def = _def;
			}
			break;
		case OUTPUT_TYPE_H265:
			{
				NALU_STREAM_DEF _def = {
						.SC = { 0x00, 0x00, 0x00, 0x01 },
						.SOI = { 0x48, 0x45 }, //'H', 'E'
						.EOI = { 0x56, 0x43 }, //'V', 'C'
						.SEI_CODE = 40 << 1,
				};
				def = _def;
			}
			break;
		case OUTPUT_TYPE_H264:
			{
				NALU_STREAM_DEF _def = {
						.SC = { 0x00, 0x00, 0x00, 0x01 },
						.SOI = { 0x4E, 0x41 }, //'N', 'A'
						.EOI = { 0x4C, 0x55 }, //'L', 'U'
						.SEI_CODE = 6,
				};
				def = _def;
			}
			break;
		case OUTPUT_TYPE_MJPEG:
			{
				NALU_STREAM_DEF _def = {
						.SC = { 0x00, 0x00, 0x00, 0x01 },
						.SOI = { 0xFF, 0xD8 },
						.EOI = { 0xFF, 0xD9 },
						.SEI_CODE = 0,
				};
				def = _def;
			}
			break;
		}

		switch (frame->output_type){
		case OUTPUT_TYPE_I420:
		case OUTPUT_TYPE_H265:
		case OUTPUT_TYPE_H264:
			//header pack
			send_sei(frame, frame_info, def);
			rtp_flush(state->rtp);
			send_image(data, data_len);
			rtp_sendpacket(state->rtp, def.EOI, sizeof(def.EOI), PT_CAM_BASE);
			rtp_flush(state->rtp);
			break;
		case OUTPUT_TYPE_MJPEG:
			//header pack
			send_xmp(frame, frame_info, def);
			rtp_flush(state->rtp);
			{
				int cur = 0;
				for(;cur<data_len;cur++){
					if(data[cur+0] == def.SOI[0] && data[cur+1] == def.SOI[1])
					{
						break;
					}
				}
				for(;cur<data_len;data_len--){
					if(data[data_len-2] == def.EOI[0] && data[data_len-1] == def.EOI[1])
					{
						break;
					}
				}
				send_image(data+(cur+2), (data_len-2)-(cur+2));
			}
			rtp_sendpacket(state->rtp, def.EOI, sizeof(def.EOI), PT_CAM_BASE);
			rtp_flush(state->rtp);
			break;
		}

		if (frame->output_fd > 0) {
			switch (frame->output_type){
			case OUTPUT_TYPE_I420:
				frame->output_start = true;
				if (frame->output_start) {
					write(frame->output_fd, def.SC, 4);
					write(frame->output_fd, data + 4, data_len - 4);
				}
				break;
			case OUTPUT_TYPE_H265:
				if (!frame->output_start) {
					if (((data[4] & 0x7e) >> 1) == 32) { // wait for vps
						printf("output_start\n");
						frame->output_start = true;
					}
				}
				if (frame->output_start) {
					write(frame->output_fd, def.SC, 4);
					write(frame->output_fd, data + 4, data_len - 4);
				}
				break;
			case OUTPUT_TYPE_H264:
				if (!frame->output_start) {
					if ((data[4] & 0x1f) == 7) { // wait for sps
						printf("output_start\n");
						frame->output_start = true;
					}
				}
				if (frame->output_start) {
					write(frame->output_fd, def.SC, 4);
					write(frame->output_fd, data + 4, data_len - 4);
				}
				break;
			case OUTPUT_TYPE_MJPEG:
				frame->output_start = true;
				if (frame->output_start) {
					write(frame->output_fd, data, data_len);
				}
				break;
			}
		}
		if (lg_debug_dump) {
			switch (frame->output_type){
			case OUTPUT_TYPE_I420:
				break;
			case OUTPUT_TYPE_H265:
				break;
			case OUTPUT_TYPE_H264:
				break;
			case OUTPUT_TYPE_MJPEG:
				if (data[0] == 0xFF && data[1] == 0xD8) { //
					char path[256];
					sprintf(path, "/tmp/debug_%03d.jpeg", lg_debug_dump_num++);
					lg_debug_dump_fd = open(path, //
							O_CREAT | O_WRONLY | O_TRUNC, /*  */
							S_IRUSR | S_IWUSR | /* rw */
							S_IRGRP | S_IWGRP | /* rw */
							S_IROTH | S_IXOTH);
				}
				if (lg_debug_dump_fd > 0) {
					write(lg_debug_dump_fd, data, data_len);
				}
				if (data[data_len - 2] == 0xFF && data[data_len - 1] == 0xD9) {
					if (lg_debug_dump_fd > 0) {
						close(lg_debug_dump_fd);
						lg_debug_dump_fd = -1;
					}
				}
				break;
			}
		}
	}
	if (frame_info) {
		free(frame_info);
	}
}
typedef struct _NALU_STREAM_DEF{
	unsigned char SC[4];
	unsigned char SOI[2];
	unsigned char EOI[2];
	unsigned char SEI_CODE;
}NALU_STREAM_DEF;
static int get_frame_info_str(FRAME_T *frame, FRAME_INFO_T *frame_info, char *buff){
	int len;
	if (frame_info) { // sei for a frame
		int server_key = frame_info->server_key.tv_sec * 1000 + frame_info->server_key.tv_usec;
		float idle_time_sec = 0;
		float frame_processed_sec = 0;
		float encoded_sec = 0;
		if (frame_info->client_key[0] != '\0') {
			struct timeval diff;
			gettimeofday(&frame_info->after_encoded, NULL);

			timersub(&frame_info->before_redraw_render_texture, &frame_info->server_key, &diff);
			idle_time_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;

			timersub(&frame_info->after_redraw_render_texture, &frame_info->before_redraw_render_texture, &diff);
			frame_processed_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;

			timersub(&frame_info->after_encoded, &frame_info->after_redraw_render_texture, &diff);
			encoded_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
		}

		uuid_t uuid;
		uuid_generate(uuid);
		char uuid_str[37];//UUID_STR_LEN
		uuid_unparse_upper(uuid, uuid_str);

		len =
				sprintf(buff,
						"<picam360:frame uuid=\"%s\" frame_id=\"%d\" frame_width=\"%d\" frame_height=\"%d\" mode=\"%s\" view_quat=\"%.3f,%.3f,%.3f,%.3f\" fov=\"%.3f\" client_key=\"%s\" server_key=\"%d\" idle_time=\"%.3f\" frame_processed=\"%.3f\" encoded=\"%.3f\" />",
						uuid_str, frame->id, frame->width, frame->height, frame->renderer->name, frame_info->view_quat.x, frame_info->view_quat.y, frame_info->view_quat.z, frame_info->view_quat.w, frame_info->fov,
						frame_info->client_key, server_key, idle_time_sec, frame_processed_sec, encoded_sec);
	} else {
		len = sprintf(buff, "<picam360:frame frame_id=\"%d\" />", frame->id);
	}
	return len;
}
static void send_xmp(FRAME_T *frame, FRAME_INFO_T *frame_info, NALU_STREAM_DEF def) {
	unsigned char header_pack[512];
	char *xmp = (char*) header_pack + sizeof(def.SOI);
	int len = 0;
	len += get_frame_info_str(frame, frame_info, xmp + 4);
	xmp[2] = (len >> 8) & 0xFF;
	xmp[3] = (len >> 0) & 0xFF;
	len += 2; //length code
	xmp[0] = 0xFF;
	xmp[1] = 0xE1;
	len += 2; //xmp code
	memcpy(header_pack, def.SOI, sizeof(def.SOI));
	len += 2; //SOI
	rtp_sendpacket(state->rtp, header_pack, len, PT_CAM_BASE);
}
static void send_sei(FRAME_T *frame, FRAME_INFO_T *frame_info, NALU_STREAM_DEF def) {
	unsigned char header_pack[512];
	char *sei = (char*) header_pack + sizeof(def.SOI);
	int len = 0;
	sei[4] = def.SEI_CODE; //nal_type:sei
	len += 1; //nal header
	len += get_frame_info_str(frame, frame_info, sei + 5);
	sei[0] = (len >> 24) & 0xFF;
	sei[1] = (len >> 16) & 0xFF;
	sei[2] = (len >> 8) & 0xFF;
	sei[3] = (len >> 0) & 0xFF;
	len += 4; //start code
	memcpy(header_pack, def.SOI, sizeof(def.SOI));
	len += 2; //SOI
	rtp_sendpacket(state->rtp, header_pack, len, PT_CAM_BASE);
}
static void send_image(unsigned char *data, unsigned int data_len) {
	for (int j = 0; j < data_len;) {
		int len;
		if (j + RTP_MAXPAYLOADSIZE < data_len) {
			len = RTP_MAXPAYLOADSIZE;
		} else {
			len = data_len - j;
		}
		rtp_sendpacket(state->rtp, data + j, len, PT_CAM_BASE);
		j += len;
	}
}
