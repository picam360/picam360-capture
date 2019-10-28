static void get_info_str(char *buff, int buff_len);


static void get_info_str(char *buff, int buff_len) {
	int len = 0;
	{ //View
		float north;
		VECTOR4D_T quat = state->plugin_host.get_view_quaternion();
		quaternion_get_euler(quat, &north, NULL, NULL, EULER_SEQUENCE_YXZ);
		len += snprintf(buff + len, buff_len - len, "View   : Tmp %.1f degC, N %.1f", state->plugin_host.get_view_temperature(), north * 180 / M_PI);
	}
	{ //Vehicle
		float north;
		VECTOR4D_T quat = state->plugin_host.get_camera_quaternion(-1);
		quaternion_get_euler(quat, &north, NULL, NULL, EULER_SEQUENCE_YXZ);
		len += snprintf(buff + len, buff_len - len, "\nVehicle: Tmp %.1f degC, N %.1f, rx %.1f Mbps, fps %.1f:%.1f skip %.0f:%.0f", state->plugin_host.get_camera_temperature(), north * 180 / M_PI,
				lg_cam_bandwidth, lg_cam_fps[0], lg_cam_fps[1], lg_cam_frameskip[0], lg_cam_frameskip[1]);
	}
	for (int i = 0; state->plugins[i] != NULL; i++) {
		if (state->plugins[i]->get_info) {
			char *info = state->plugins[i]->get_info(state->plugins[i]->user_data);
			if (info) {
				len += snprintf(buff + len, buff_len - len, "\n%s", info);
			}
		}
	}
	len += snprintf(buff + len, buff_len - len, "\n");
}

#if (1) //status block

#define STATUS_VAR(name) lg_status_ ## name
#define STATUS_INIT(plugin_host, prefix, name) STATUS_VAR(name) = new_status(prefix #name); \
                                               (plugin_host)->add_status(STATUS_VAR(name));
#define WATCH_VAR(name) lg_watch_ ## name
#define WATCH_INIT(plugin_host, prefix, name) WATCH_VAR(name) = new_status(prefix #name); \
                                               (plugin_host)->add_watch(WATCH_VAR(name));
//status to downstream
static STATUS_T *STATUS_VAR(ack_command_id);
static STATUS_T *STATUS_VAR(last_frame_id);
static STATUS_T *STATUS_VAR(quaternion);
static STATUS_T *STATUS_VAR(north);
static STATUS_T *STATUS_VAR(info);
static STATUS_T *STATUS_VAR(menu);
//watch status from upstream
static STATUS_T *WATCH_VAR(ack_command_id);
static STATUS_T *WATCH_VAR(quaternion);
static STATUS_T *WATCH_VAR(compass);
static STATUS_T *WATCH_VAR(temperature);
static STATUS_T *WATCH_VAR(bandwidth);
static STATUS_T *WATCH_VAR(cam_fps);
static STATUS_T *WATCH_VAR(cam_frameskip);

static void status_release(void *user_data) {
	free(user_data);
}
static void status_get_value(void *user_data, char *buff, int buff_len) {
	STATUS_T *status = (STATUS_T*) user_data;
	if (status == STATUS_VAR(ack_command_id)) {
		snprintf(buff, buff_len, "%d", lg_ack_command_id_downstream);
	} else if (status == STATUS_VAR(last_frame_id)) {
		snprintf(buff, buff_len, "%d", state->last_frame_id);
	} else if (status == STATUS_VAR(quaternion)) {
		VECTOR4D_T quat = state->mpu->get_quaternion(state->mpu);
		snprintf(buff, buff_len, "%f,%f,%f,%f", quat.x, quat.y, quat.z, quat.w);
	} else if (status == STATUS_VAR(north)) {
		float north = state->mpu->get_north(state->mpu);
		snprintf(buff, buff_len, "%f", north);
	} else if (status == STATUS_VAR(info)) {
		get_info_str(buff, buff_len);
	} else if (status == STATUS_VAR(menu)) {
		get_menu_str(buff, buff_len);
	}
}
static void status_set_value(void *user_data, const char *value) {
	STATUS_T *status = (STATUS_T*) user_data;
	if (status == WATCH_VAR(ack_command_id)) {
		sscanf(value, "%d", &lg_ack_command_id_upstream);
	} else if (status == WATCH_VAR(quaternion)) {
		VECTOR4D_T vec = { };
		sscanf(value, "%f,%f,%f,%f", &vec.x, &vec.y, &vec.z, &vec.w);
		state->plugin_host.set_camera_quaternion(-1, vec);
	} else if (status == WATCH_VAR(compass)) {
		VECTOR4D_T vec = { };
		sscanf(value, "%f,%f,%f,%f", &vec.x, &vec.y, &vec.z, &vec.w);
		state->plugin_host.set_camera_compass(vec);
	} else if (status == WATCH_VAR(temperature)) {
		float temperature = 0;
		sscanf(value, "%f", &temperature);
		state->plugin_host.set_camera_temperature(temperature);
	} else if (status == WATCH_VAR(bandwidth)) {
		sscanf(value, "%f", &lg_cam_bandwidth);
	} else if (status == WATCH_VAR(cam_fps)) {
		sscanf(value, "%f,%f", &lg_cam_fps[0], &lg_cam_fps[1]);
	} else if (status == WATCH_VAR(cam_frameskip)) {
		sscanf(value, "%f,%f", &lg_cam_frameskip[0], &lg_cam_frameskip[1]);
	}
}

static STATUS_T *new_status(const char *name) {
	STATUS_T *status = (STATUS_T*) malloc(sizeof(STATUS_T));
	strcpy(status->name, name);
	status->get_value = status_get_value;
	status->set_value = status_set_value;
	status->release = status_release;
	status->user_data = status;
	return status;
}

static void init_status() {
	STATUS_INIT(&state->plugin_host, "", ack_command_id);
	STATUS_INIT(&state->plugin_host, "", last_frame_id);
	STATUS_INIT(&state->plugin_host, "", quaternion);
	STATUS_INIT(&state->plugin_host, "", north);
	STATUS_INIT(&state->plugin_host, "", info);
	STATUS_INIT(&state->plugin_host, "", menu);
	WATCH_INIT(&state->plugin_host, UPSTREAM_DOMAIN, ack_command_id);
	WATCH_INIT(&state->plugin_host, UPSTREAM_DOMAIN, quaternion);
	WATCH_INIT(&state->plugin_host, UPSTREAM_DOMAIN, compass);
	WATCH_INIT(&state->plugin_host, UPSTREAM_DOMAIN, temperature);
	WATCH_INIT(&state->plugin_host, UPSTREAM_DOMAIN, bandwidth);
	WATCH_INIT(&state->plugin_host, UPSTREAM_DOMAIN, cam_fps);
	WATCH_INIT(&state->plugin_host, UPSTREAM_DOMAIN, cam_frameskip);
}

#endif //status block
