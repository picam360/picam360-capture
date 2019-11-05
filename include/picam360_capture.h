#pragma once

#include <stdbool.h>
#ifdef USE_GLES
#include "GLES2/gl2.h"
#include "GLES2/gl2ext.h"
#else
#include <GL/glew.h>
#include <GLFW/glfw3.h>
//#include "GL/gl.h"
//#include "GL/glut.h"
//#include "GL/glext.h"
#endif
#include <pthread.h>
#include "mrevent.h"
#include "rtp.h"
#include "egl_handler.h"

#include "picam360_capture_plugin.h"

#define MAX_OSTREAM_NUM 16
#define TEXTURE_BUFFER_NUM 2
#define MAX_OPERATION_NUM 7
#define MAX_QUATERNION_QUEUE_COUNT 128 //keep 1280ms
#define QUATERNION_QUEUE_RES 20 //50hz

enum INPUT_MODE {
	INPUT_MODE_NONE, INPUT_MODE_CAM, INPUT_MODE_FILE
};
enum OUTPUT_MODE {
	OUTPUT_MODE_NONE, OUTPUT_MODE_STILL, OUTPUT_MODE_VIDEO, OUTPUT_MODE_STREAM
};
enum OUTPUT_TYPE {
	OUTPUT_TYPE_NONE, OUTPUT_TYPE_JPEG, OUTPUT_TYPE_MJPEG, OUTPUT_TYPE_H264, OUTPUT_TYPE_H265, OUTPUT_TYPE_I420, OUTPUT_TYPE_RGB24
};
enum OPERATION_MODE {
	OPERATION_MODE_NONE, OPERATION_MODE_BOARD, OPERATION_MODE_WINDOW, OPERATION_MODE_PICAM360MAP, OPERATION_MODE_FISHEYE, OPERATION_MODE_CALIBRATION
};

struct _PICAM360CAPTURE_T;

typedef struct _OPTIONS_T {
	float sharpness_gain;
	float color_offset;
	float overlap;
	float cam_offset_pitch[MAX_CAM_NUM]; // x axis
	float cam_offset_yaw[MAX_CAM_NUM]; // y axis
	float cam_offset_roll[MAX_CAM_NUM]; // z axis
	float cam_offset_x[MAX_CAM_NUM];
	float cam_offset_y[MAX_CAM_NUM];
	float cam_horizon_r[MAX_CAM_NUM];
	float cam_aov[MAX_CAM_NUM];

	char config_ex_filepath[256];
	bool config_ex_enabled;
	float cam_offset_x_ex[MAX_CAM_NUM];
	float cam_offset_y_ex[MAX_CAM_NUM];
	float cam_horizon_r_ex[MAX_CAM_NUM];

	float view_offset_pitch; // x axis
	float view_offset_yaw; // y axis
	float view_offset_roll; // z axis

	int rtp_rx_port;
	enum RTP_SOCKET_TYPE rtp_rx_type;
	char rtp_tx_ip[256];
	int rtp_tx_port;
	enum RTP_SOCKET_TYPE rtp_tx_type;

	int rtcp_rx_port;
	enum RTP_SOCKET_TYPE rtcp_rx_type;
	char rtcp_tx_ip[256];
	int rtcp_tx_port;
	enum RTP_SOCKET_TYPE rtcp_tx_type;

	bool is_samplerExternalOES;
} OPTIONS_T;

typedef struct _LIST_T {
	void *value;
	struct _LIST_T *next;
} LIST_T;

typedef struct _FRAME_INFO_T {
	VECTOR4D_T view_quat;
	float fov;
	char client_key[256];
	struct timeval server_key;
	struct timeval before_redraw_render_texture;
	struct timeval after_redraw_render_texture;
	struct timeval after_encoded;
} FRAME_INFO_T;

typedef struct _VOSTREAM_T {
	int id;
	GLuint framebuffer;
	GLuint texture;
	uint8_t *img_buff;
	uint32_t img_width;
	uint32_t img_height;
	uint32_t width;
	uint32_t height;
	int frame_num;
	double frame_elapsed;
	bool is_recording;
	bool stereo;

	enum OUTPUT_MODE output_mode;
	enum OUTPUT_TYPE output_type;
	char output_filepath[256];
	int output_fd;
	bool output_start;
	bool double_size;

	float kbps;
	float fps;
	struct timeval last_updated;

	// for latency cal
	char client_key[256];
	struct timeval server_key;

	void *custom_data;
	//event
	void (*after_processed_callback)(struct _PICAM360CAPTURE_T *, struct _VOSTREAM_T *);
	void (*befor_deleted_callback)(struct _PICAM360CAPTURE_T *, struct _VOSTREAM_T *);

	VSTREAMER_T *vstreamer;
} VOSTREAM_T;
typedef struct _MODEL_T {
	void *program;
	GLuint vbo;
	GLuint vbo_nop;
	GLuint vao;
} MODEL_T;
typedef struct _PICAM360CAPTURE_T {
	PLUGIN_HOST_T plugin_host;
	uuid_t uuid;
	pthread_mutex_t mutex;
	char config_filepath[512];
	int split;
	bool preview;
	bool conf_sync;
	bool video_direct;
	char vistream_def[256];
	char aistream_def[256];
	int32_t screen_width;
	int32_t screen_height;
	uint32_t cam_width;
	uint32_t cam_height;

	EGL_HANDLER_T egl_handler;

	int active_cam;
	int num_of_cam;
	pthread_t thread[MAX_CAM_NUM];
	GLuint cam_texture[MAX_CAM_NUM][TEXTURE_BUFFER_NUM]; //double buffer
	int cam_texture_cur[MAX_CAM_NUM];
	GLuint logo_texture;
// model rotation vector and direction
	GLfloat rot_angle_x_inc;
	GLfloat rot_angle_y_inc;
	GLfloat rot_angle_z_inc;
// current model rotation angles
	GLfloat rot_angle_x;
	GLfloat rot_angle_y;
	GLfloat rot_angle_z;
// current distance from camera
	GLfloat distance;
	GLfloat distance_inc;

	//logo
	PICAM360_IMAGE_T logo_image;

	MREVENT_T request_frame_event[MAX_CAM_NUM];
	MREVENT_T arrived_frame_event[MAX_CAM_NUM];
	enum INPUT_MODE input_mode;
	char input_filepath[256];
	int input_file_size;
	int input_file_cur;
	bool frame_sync;
	bool output_raw;
	char output_raw_filepath[256];

	//for unif matrix
	//euler angles
	float refraction;
	float camera_pitch; // x axis
	float camera_yaw; // y axis
	float camera_roll; // z axis
	VECTOR4D_T camera_quaternion[MAX_CAM_NUM + 1];
	VECTOR4D_T camera_compass;
	float camera_horizon_r_bias;
	float camera_temperature;
	float camera_north;
	bool camera_coordinate_from_device;
	char default_view_coordinate_mode[64];

	MODEL_T model_data[MAX_OPERATION_NUM];
	pthread_mutex_t texture_mutex;
	pthread_mutex_t texture_size_mutex;

	VSTREAMER_T *vistreams[MAX_CAM_NUM];
	VSTREAMER_T *aistream;

	unsigned int last_vostream_id;
	VOSTREAM_T *vostreams[MAX_OSTREAM_NUM];

	pthread_mutex_t cmd_list_mutex;
	LIST_T *cmd_list;
	LIST_T *cmd2upstream_list;

	MENU_T *menu;
	bool menu_visible;

	//rtp
	RTP_T *rtp;
	RTP_T *rtcp;
	float rtp_play_speed;

	char command[256];
	int command_id;
	int ack_command_id_downstream;
	int ack_command_id_upstream;

	char mpu_name[64];
	MPU_T *mpu;
	int quaternion_queue_cur;
	VECTOR4D_T quaternion_queue[MAX_QUATERNION_QUEUE_COUNT];

	char **plugin_paths;
	PLUGIN_T **plugins;
	MPU_FACTORY_T **mpu_factories;
	VSTREAMER_FACTORY_T **vstreamer_factories;
	STATUS_T **statuses;
	STATUS_T **watches;
	struct _OPTIONS_T options;
} PICAM360CAPTURE_T;
