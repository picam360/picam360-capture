#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#if __linux
#include <sys/prctl.h>
#endif

#ifdef USE_GLES
#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#else
#include <GL/glew.h>
#include <GLFW/glfw3.h>
//#include "GL/gl.h"
//#include "GL/glut.h"
//#include "GL/glext.h"
#endif

#include "gl_program.h"
#include "glsl/window_fsh.h"
#include "glsl/window_vsh.h"

#include "window_gl_renderer.h"
#include "mrevent.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PLUGIN_NAME "window_gl_renderer"
#define RENDERER_NAME "WINDOW"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

#define BUFFER_NUM 4
typedef struct _window_gl_renderer {
	VSTREAMER_T super;

	char img_type[4];
	int width;
	int height;
	int stereo;
	int fov;
	VECTOR4D_T view_quat;

	void *program_obj;
	GLuint vbo;
	GLuint vbo_nop;
	GLuint vao;

	int id;
	bool run;
	pthread_t streaming_thread;
	int framecount;

	MREVENT_T frame_ready;
	EGLImageKHR egl_images[BUFFER_NUM];
	PICAM360_IMAGE_T frame_buffers[BUFFER_NUM];
	uint8_t xmp_buffers[BUFFER_NUM][RTP_MAXPAYLOADSIZE];
} window_gl_renderer;

int spherewindow_mesh(float theta_degree, int phi_degree, int num_of_steps, GLuint *vbo_out, GLuint *n_out, GLuint *vao_out) {
	GLuint vbo;

	int n = 2 * (num_of_steps + 1) * num_of_steps;
	float points[4 * n];

	float theta = theta_degree * M_PI / 180.0;
	float phi = phi_degree * M_PI / 180.0;

	float start_x = -tan(theta / 2);
	float start_y = -tan(phi / 2);

	float end_x = tan(theta / 2);
	float end_y = tan(phi / 2);

	float step_x = (end_x - start_x) / num_of_steps;
	float step_y = (end_y - start_y) / num_of_steps;

	int idx = 0;
	int i, j;
	for (i = 0; i < num_of_steps; i++) {	//x
		for (j = 0; j <= num_of_steps; j++) {	//y
			{
				float x = start_x + step_x * i;
				float y = start_y + step_y * j;
				float z = 1.0;
				float len = sqrt(x * x + y * y + z * z);
				points[idx++] = x / len;
				points[idx++] = y / len;
				points[idx++] = z / len;
				points[idx++] = 1.0;
				//printf("x=%f,y=%f,z=%f,w=%f\n", points[idx - 4],
				//		points[idx - 3], points[idx - 2], points[idx - 1]);
			}
			{
				float x = start_x + step_x * (i + 1);
				float y = start_y + step_y * j;
				float z = 1.0;
				float len = sqrt(x * x + y * y + z * z);
				points[idx++] = x / len;
				points[idx++] = y / len;
				points[idx++] = z / len;
				points[idx++] = 1.0;
				//printf("x=%f,y=%f,z=%f,w=%f\n", points[idx - 4],
				//		points[idx - 3], points[idx - 2], points[idx - 1]);
			}
		}
	}

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * n, points, GL_STATIC_DRAW);

#ifdef USE_VAO
	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindVertexArray(0);

	if (vao_out != NULL)
	*vao_out = vao;
#endif

	if (vbo_out != NULL)
	*vbo_out = vbo;
	if (n_out != NULL)
	*n_out = n;

	return 0;
}

static void release(void *obj) {
	window_gl_renderer *_this = (window_gl_renderer*) obj;

	if (_this->run) {
		_this->super.stop(&_this->super);
	}

	free(obj);
}

static int get_frame_info_str(window_gl_renderer *_this,
		PICAM360_IMAGE_T *image, char *buff, int *buff_sizse) {
	int len = 0;
	char uuid_str[37]; //UUID_STR_LEN
	uuid_unparse_upper(image->uuid, uuid_str);

	len += sprintf(buff + len, "<picam360:frame ");
	len += sprintf(buff + len, "uuid=\"%s\" ", uuid_str);
	len += sprintf(buff + len, "timestamp=\"%ld,%ld\" ",
			image->timestamp.tv_sec, image->timestamp.tv_usec);
	len += sprintf(buff + len, "frame_id=\"%d\" ", _this->id);
	len += sprintf(buff + len, "frame_width=\"%d\" ", _this->width);
	len += sprintf(buff + len, "frame_height=\"%d\" ", _this->height);
	len += sprintf(buff + len, "mode=\"%s\" ", _this->super.name);
	len += sprintf(buff + len, "fov=\"%d\" ", _this->fov);
	len += sprintf(buff + len, "view_quat=\"%.3f,%.3f,%.3f,%.3f\" ",
			_this->view_quat.x, _this->view_quat.y, _this->view_quat.z,
			_this->view_quat.w);
	len += sprintf(buff + len, "/>");

	*buff_sizse = len;

	return 0;
}
static void render(window_gl_renderer *_this) {

	int fov = 120;

	int program = GLProgram_GetId(_this->program_obj);

	int frame_width = (_this->stereo) ? _this->width / 2 : _this->width;
	int frame_height = _this->height;

	glUseProgram(program);

	glViewport(0, 0, frame_width, frame_height);

	{ // bind texture
//		glActiveTexture(GL_TEXTURE0);
//		glBindTexture(GL_TEXTURE_2D, _this->logo_texture);
//		glUniform1i(glGetUniformLocation(program, "logo_texture"), 0);

//		int cam_texture[MAX_CAM_NUM];
//		for (int i = 0; i < state->num_of_cam; i++) {
//			cam_texture[i] = i + 1;
//			glActiveTexture(GL_TEXTURE1 + i);
//			glBindTexture(GL_TEXTURE_2D, state->cam_texture[i][state->cam_texture_cur[i]]);
//		}
//		glUniform1iv(glGetUniformLocation(program, "cam_texture"), state->num_of_cam, cam_texture);
	}

//	{ //cam_attitude //depth axis is z, vertical asis is y
//		float cam_attitude[16 * MAX_CAM_NUM];
//		float view_matrix[16];
//		float north_matrix[16];
//		float world_matrix[16];
//
//		{ // Rv : view
//			mat4_identity(view_matrix);
//			mat4_fromQuat(view_matrix, view_quat.ary);
//			mat4_invert(view_matrix, view_matrix);
//		}
//
//		{ // Rn : north
//			mat4_identity(north_matrix);
//			float north = state->plugin_host.get_view_north();
//			mat4_rotateY(north_matrix, north_matrix, north * M_PI / 180);
//		}
//
//		{ // Rw : view coodinate to world coodinate and view heading to ground initially
//			mat4_identity(world_matrix);
//			mat4_rotateX(world_matrix, world_matrix, -M_PI / 2);
//		}
//
//		for (int i = 0; i < state->num_of_cam; i++) {
//			float *unif_matrix = cam_attitude + 16 * i;
//			float cam_matrix[16];
//			{ // Rc : cam orientation
//				mat4_identity(cam_matrix);
//				if (state->camera_coordinate_from_device) {
//					mat4_fromQuat(cam_matrix, state->camera_quaternion[i].ary);
//				} else {
//					//euler Y(yaw)X(pitch)Z(roll)
//					mat4_rotateZ(cam_matrix, cam_matrix, state->camera_roll);
//					mat4_rotateX(cam_matrix, cam_matrix, state->camera_pitch);
//					mat4_rotateY(cam_matrix, cam_matrix, state->camera_yaw);
//				}
//			}
//
//			{ // Rco : cam offset  //euler Y(yaw)X(pitch)Z(roll)
//				float cam_offset_matrix[16];
//				mat4_identity(cam_offset_matrix);
//				mat4_rotateZ(cam_offset_matrix, cam_offset_matrix, state->options.cam_offset_roll[i]);
//				mat4_rotateX(cam_offset_matrix, cam_offset_matrix, state->options.cam_offset_pitch[i]);
//				mat4_rotateY(cam_offset_matrix, cam_offset_matrix, state->options.cam_offset_yaw[i]);
//				mat4_invert(cam_offset_matrix, cam_offset_matrix);
//				mat4_multiply(cam_matrix, cam_matrix, cam_offset_matrix); // Rc'=RcoRc
//			}
//
//			{ //RcRv(Rc^-1)RcRw
//				mat4_identity(unif_matrix);
//				mat4_multiply(unif_matrix, unif_matrix, world_matrix); // Rw
//				mat4_multiply(unif_matrix, unif_matrix, view_matrix); // RvRw
//				//mat4_multiply(unif_matrix, unif_matrix, north_matrix); // RnRvRw
//				mat4_multiply(unif_matrix, unif_matrix, cam_matrix); // RcRnRvRw
//			}
//			mat4_transpose(unif_matrix, unif_matrix); // this mat4 library is row primary, opengl is column primary
//		}
//		glUniformMatrix4fv(glGetUniformLocation(program, "cam_attitude"), state->num_of_cam, GL_FALSE, (GLfloat*) cam_attitude);
//	}
//	{ //cam_options
//		float cam_offset_yaw[MAX_CAM_NUM];
//		float cam_offset_x[MAX_CAM_NUM];
//		float cam_offset_y[MAX_CAM_NUM];
//		float cam_horizon_r[MAX_CAM_NUM];
//		float cam_aov[MAX_CAM_NUM];
//		for (int i = 0; i < state->num_of_cam; i++) {
//			cam_offset_x[i] = state->options.cam_offset_x[i];
//			cam_offset_y[i] = state->options.cam_offset_y[i];
//			cam_horizon_r[i] = state->options.cam_horizon_r[i];
//			cam_aov[i] = state->options.cam_aov[i];
//			if (state->options.config_ex_enabled) {
//				cam_offset_x[i] += state->options.cam_offset_x_ex[i];
//				cam_offset_y[i] += state->options.cam_offset_y_ex[i];
//				cam_horizon_r[i] += state->options.cam_horizon_r_ex[i];
//			}
//			cam_horizon_r[i] *= state->camera_horizon_r_bias;
//			cam_aov[i] /= state->refraction;
//		}
//		glUniform1fv(glGetUniformLocation(program, "cam_offset_x"), state->num_of_cam, cam_offset_x);
//		glUniform1fv(glGetUniformLocation(program, "cam_offset_y"), state->num_of_cam, cam_offset_y);
//		glUniform1fv(glGetUniformLocation(program, "cam_horizon_r"), state->num_of_cam, cam_horizon_r);
//		glUniform1fv(glGetUniformLocation(program, "cam_aov"), state->num_of_cam, cam_aov);
//
//		glUniform1i(glGetUniformLocation(program, "active_cam"), state->active_cam);
//		glUniform1i(glGetUniformLocation(program, "num_of_cam"), state->num_of_cam);
//	}
//
//	//these should be into each plugin
//	//Load in the texture and thresholding parameters.
//	glUniform1f(glGetUniformLocation(program, "split"), state->split);
//	glUniform1f(glGetUniformLocation(program, "pixel_size"), 1.0 / state->cam_width);
//
//	glUniform1f(glGetUniformLocation(program, "cam_aspect_ratio"), (float) state->cam_width / (float) state->cam_height);
//	glUniform1f(glGetUniformLocation(program, "frame_aspect_ratio"), (float) frame_width / (float) frame_height);
//
//	glUniform1f(glGetUniformLocation(program, "sharpness_gain"), state->options.sharpness_gain);
//	glUniform1f(glGetUniformLocation(program, "color_offset"), state->options.color_offset);
//	glUniform1f(glGetUniformLocation(program, "color_factor"), 1.0 / (1.0 - state->options.color_offset));
//	glUniform1f(glGetUniformLocation(program, "overlap"), state->options.overlap);
//
//	glDisable(GL_BLEND);
//	glEnable(GL_CULL_FACE);

	glBindBuffer(GL_ARRAY_BUFFER, _this->vbo);

	float fov_rad = fov * M_PI / 180.0;
	float scale = 1.0 / tan(fov_rad / 2);
	glUniform1f(glGetUniformLocation(program, "scale"), scale);

#ifdef USE_VAO
	glBindVertexArray(_this->vao);
#else
	GLuint loc = glGetAttribLocation(program, "vPosition");
	glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(loc);
#endif

	glDrawArrays(GL_TRIANGLE_STRIP, 0, _this->vbo_nop);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
#ifdef USE_VAO
	glBindVertexArray(0);
#else
	glDisableVertexAttribArray(loc);
#endif
}

static void* streaming_thread_fnc(void *obj) {
	pthread_setname_np(pthread_self(), "RENDERING");

	window_gl_renderer *_this = (window_gl_renderer*) obj;

	while (_this->run) {
		if (_this->super.pre_streamer == NULL) {
			usleep(100 * 1000);
			continue;
		}
		int ret;
		int num = MAX_CAM_NUM;
		PICAM360_IMAGE_T *images[MAX_CAM_NUM];
		ret = _this->super.pre_streamer->get_image(_this->super.pre_streamer,
				images, &num, 100 * 1000);
		if (ret != 0) {
			continue;
		}

		int cur = _this->framecount % BUFFER_NUM;
		PICAM360_IMAGE_T *frame = &_this->frame_buffers[cur];
		frame->meta = _this->xmp_buffers[cur];

		if (frame->id[0] == 0) {
			//allocate memory
		}
		uuid_generate(frame->uuid);
		frame->timestamp = images[0]->timestamp;
		if (_this->img_type[0] == 'R') {
			memcpy(frame->img_type, "RGBA", 4);
		} else {
			memcpy(frame->img_type, "I420", 4);
		}
		frame->mem_type = PICAM360_MEMORY_TYPE_EGL;
		frame->meta_size = RTP_MAXPAYLOADSIZE;
		get_frame_info_str(_this, frame, (char*) frame->meta,
				(int*) &frame->meta_size);

		render(_this);

		_this->framecount++;
		mrevent_trigger(&_this->frame_ready);

		{
			static int count = 0;
			static float elapsed_sec = 0;
			float _elapsed_sec;
			struct timeval diff;

			struct timeval now;
			gettimeofday(&now, NULL);

			timersub(&now, &images[0]->timestamp, &diff);
			_elapsed_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
			if ((count % 200) == 0) {
				printf("window_tegra_render : %f\n", _elapsed_sec);
			}
			count++;
		}
	}

	return NULL;
}

static void start(void *user_data) {
	window_gl_renderer *_this = (window_gl_renderer*) user_data;

	if (_this->super.next_streamer) {
		_this->super.next_streamer->start(_this->super.next_streamer);
	}

	_this->run = true;
	pthread_create(&_this->streaming_thread, NULL, streaming_thread_fnc,
			user_data);
}

static void stop(void *user_data) {
	window_gl_renderer *_this = (window_gl_renderer*) user_data;

	if (_this->run) {
		_this->run = false;
		pthread_join(_this->streaming_thread, NULL);
	}

	if (_this->super.next_streamer) {
		_this->super.next_streamer->stop(_this->super.next_streamer);
	}
}

static int get_image(void *obj, PICAM360_IMAGE_T **image_p, int *num_p,
		int wait_usec) {
	window_gl_renderer *_this = (window_gl_renderer*) obj;

	int res = mrevent_wait(&_this->frame_ready, wait_usec);
	if (res != 0) {
		return -1;
	} else {
		mrevent_reset(&_this->frame_ready);
	}

	int cur = (_this->framecount - 1) % BUFFER_NUM;

	*image_p = &_this->frame_buffers[cur];
	*num_p = 1;

	return 0;
}

static int set_param(void *obj, const char *param, const char *value_str) {
	window_gl_renderer *_this = (window_gl_renderer*) obj;
	if (strcmp(param, "id") == 0) {
		int value = 0;
		sscanf(value_str, "%d", &value);
		_this->id = MAX(0, value);
	} else if (strcmp(param, "width") == 0) {
		int value = 0;
		sscanf(value_str, "%d", &value);
		_this->width = MAX(0, MIN(value, 2048));
	} else if (strcmp(param, "height") == 0) {
		int value = 0;
		sscanf(value_str, "%d", &value);
		_this->height = MAX(0, MIN(value, 2048));
	} else if (strcmp(param, "fov") == 0) {
		int value = 0;
		sscanf(value_str, "%d", &value);
		_this->fov = MAX(0, MIN(value, 180));
	} else if (strcmp(param, "view_quat") == 0) {
		float x, y, z, w;
		int num = sscanf(value_str, "%f,%f,%f,%f", &x, &y, &z, &w);
		if (num == 4) {
			VECTOR4D_T value = { .ary = { x, y, z, w } };
			_this->view_quat = value;
		}
	} else if (strcmp(param, "img_type") == 0) {
		char img_type[5] = { };
		sscanf(value_str, "%4s", img_type);
		memcpy(_this->img_type, img_type, 4);
	}
}

static int get_param(void *obj, const char *param, char *value, int size) {

}

static void create_renderer(void *user_data, VSTREAMER_T **out_renderer) {
	VSTREAMER_FACTORY_T *factory = (VSTREAMER_FACTORY_T*) user_data;
	VSTREAMER_T *renderer = (VSTREAMER_T*) malloc(
			sizeof(window_gl_renderer));
	memset(renderer, 0, sizeof(window_gl_renderer));
	strcpy(renderer->name, factory->name);
	renderer->release = release;
	renderer->start = start;
	renderer->stop = stop;
	renderer->set_param = set_param;
	renderer->get_param = get_param;
	renderer->get_image = get_image;
	renderer->user_data = renderer;

	window_gl_renderer *_private = (window_gl_renderer*) renderer;
	mrevent_init(&_private->frame_ready);
	_private->width = 256;
	_private->height = 256;

	_private->fov = 120;
	_private->view_quat = quaternion_init();
//	{ // logo
//		PICAM360_IMAGE_T pcimage = { };
//		lg_plugin_host->get_logo_image(&pcimage);
//		IMAGE_T logo_image;
//		if (memcmp(pcimage.img_type, "RGB\0", 4) == 0) {
//			logo_image.width = pcimage.width[0];
//			logo_image.height = pcimage.height[0];
//			logo_image.stride = pcimage.stride[0];
//			logo_image.pixels = pcimage.pixels[0];
//			_private->instance = window_cuda_new(&logo_image);
//		} else if (memcmp(pcimage.img_type, "RGBA", 4) == 0) {
//			logo_image.width = pcimage.width[0];
//			logo_image.height = pcimage.height[0];
//			logo_image.stride = logo_image.width * 3;
//			logo_image.pixels = (uint8_t*) malloc(
//					logo_image.stride * logo_image.height);
//			for (int y = 0; y < logo_image.height; y++) {
//				for (int x = 0; x < logo_image.width; x++) {
//					memcpy(&logo_image.pixels[logo_image.stride * y + x * 3],
//							&pcimage.pixels[0][pcimage.stride[0] * y + x * 4],
//							3);
//				}
//			}
//			_private->instance = window_cuda_new(&logo_image);
//			free(logo_image.pixels);
//		}
//	}

	int common_cur = 0;
	char common[256];
#ifdef USE_GLES
	common_cur += sprintf(common + common_cur, "#version 100\n");
	if(false) {
		common_cur += sprintf(common + common_cur, "#extension GL_OES_EGL_image_external: require\n");
		common_cur += sprintf(common + common_cur, "#define cam_sampler2D samplerExternalOES\n");
	} else {
		common_cur += sprintf(common + common_cur, "#define cam_sampler2D sampler2D\n");
	}
#else
	common_cur += sprintf(common + common_cur, "#version 330\n");
	common_cur += sprintf(common + common_cur, "#define cam_sampler2D sampler2D\n");
#endif

	float maxfov = 150.0;
	spherewindow_mesh(maxfov, maxfov, 64, &_private->vbo, &_private->vbo_nop, &_private->vao);
	{
		const char *fsh_filepath = "/tmp/tmp.fsh";
		const char *vsh_filepath = "/tmp/tmp.vsh";
		int fsh_fd = open(fsh_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		int vsh_fd = open(vsh_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		write(fsh_fd, window_fsh, window_fsh_len);
		write(vsh_fd, window_vsh, window_vsh_len);
		close(fsh_fd);
		close(vsh_fd);
		_private->program_obj = GLProgram_new(common, vsh_filepath, fsh_filepath, true);
		remove(fsh_filepath);
		remove(vsh_filepath);
	}

	if (out_renderer) {
		*out_renderer = renderer;
	}
}

static int command_handler(void *user_data, const char *_buff) {
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id,
		uint32_t event_id) {
}

static void init_options(void *user_data, json_t *_options) {
	PLUGIN_T *plugin = (PLUGIN_T*) user_data;
	json_t *options = json_object_get(_options, PLUGIN_NAME);
	if (options == NULL) {
		return;
	}
}

static void save_options(void *user_data, json_t *_options) {
	json_t *options = json_object();
	json_object_set_new(_options, PLUGIN_NAME, options);
}

void create_window_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	lg_plugin_host = plugin_host;

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(PLUGIN_T));
		memset(plugin, 0, sizeof(PLUGIN_T));
		strcpy(plugin->name, PLUGIN_NAME);
		plugin->release = release;
		plugin->command_handler = command_handler;
		plugin->event_handler = event_handler;
		plugin->init_options = init_options;
		plugin->save_options = save_options;
		plugin->get_info = NULL;
		plugin->user_data = plugin;

		*_plugin = plugin;
	}
	{
		VSTREAMER_FACTORY_T *factory = (VSTREAMER_FACTORY_T*) malloc(
				sizeof(VSTREAMER_FACTORY_T));
		memset(factory, 0, sizeof(VSTREAMER_FACTORY_T));
		strcpy(factory->name, "WINDOW");
		factory->release = release;
		factory->create_vstreamer = create_renderer;
		factory->user_data = factory;
		lg_plugin_host->add_vstreamer_factory(factory);
	}
}

