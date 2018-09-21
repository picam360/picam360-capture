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

#include "picam360map_renderer.h"

#define PLUGIN_NAME "picam360map_renderer"
#define RENDERER_NAME "equirectangular"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

//rtp or uvc
static char lg_options_input_type[32] = { 'u', 'v', 'c' };
static char lg_options_input_codec[32] = { 'm', 'j', 'p', 'e', 'g' };

typedef struct _picam360map_renderer {
	RENDERER_T super;

	int num_of_cam;
	void *program;
	GLuint vbo;
	GLuint vbo_nop;
	GLuint vao;

	void *user_data;
} picam360map_renderer;

const char * vertex_shader = R"glsl(
#if (__VERSION__ > 120)
# define IN in
# define OUT out
#else
# define IN attribute
# define OUT varying
#endif // __VERSION
precision mediump float;
const int STEPNUM = 256;
const float STEPNUM_M1 = 255.0;

IN vec4 vPosition; //[0:1]
uniform float scale_x;
uniform float scale_y;
uniform float frame_aspect_ratio;

uniform mat4 unif_matrix;
uniform mat4 unif_matrix_1;
uniform sampler2D cam0_texture;
uniform sampler2D cam1_texture;
uniform float pixel_size;
uniform float cam_aspect_ratio;
//angular map params
uniform float r_2_pitch[STEPNUM];
//options start
uniform float sharpness_gain;
uniform float cam0_offset_yaw;
uniform float cam0_offset_x;
uniform float cam0_offset_y;
uniform float cam0_horizon_r;
uniform float cam0_aov;
uniform float cam1_offset_yaw;
uniform float cam1_offset_x;
uniform float cam1_offset_y;
uniform float cam1_horizon_r;
uniform float cam1_aov;
//options end

const float M_PI = 3.1415926535;
const float M_PI_DIV_2 = M_PI / 2.0;
const float M_PI_DIV_4 = M_PI / 4.0;
const float M_SQRT_2 = 1.4142135623;

OUT float r0;
OUT float r1;
OUT float u0;
OUT float v0;
OUT float u1;
OUT float v1;

void main(void) {
	vec4 position;
	position.xy = vPosition.xy * vec2(2.0, 2.0) + vec2(-1.0, -1.0); //[-1:1]
	gl_Position.xy = vec2(position.x, position.y * frame_aspect_ratio);
	gl_Position.zw = vec2(1.0, 1.0);

	float r = sqrt(position.x * position.x + position.y * position.y);
	if (r > M_SQRT_2) {
		r = M_SQRT_2;
	}
	float indexf = r / M_SQRT_2 * STEPNUM_M1;
	int index = int(indexf);
	float index_sub = indexf - float(index);
	float pitch_orig = r_2_pitch[index] * (1.0 - index_sub) + r_2_pitch[index + 1] * index_sub;
	float roll_orig = atan(position.y, position.x);
	if (r <= M_SQRT_2 && r > 1.0) {
		int roll_index = int(roll_orig / M_PI_DIV_2);
		float roll_base = float(roll_index) * M_PI_DIV_2 + (roll_orig > 0.0 ? M_PI_DIV_4 : -M_PI_DIV_4);
		float roll_diff = roll_orig - roll_base;
		float roll_gain = M_PI / (M_PI - 4.0 * acos(1.0 / r));
		roll_orig = roll_diff * roll_gain + roll_base;
	}
	position.x = sin(pitch_orig) * cos(roll_orig);
	position.y = sin(pitch_orig) * sin(roll_orig);
	position.z = cos(pitch_orig);
	position.w = 1.0;

	vec4 pos = unif_matrix * position;
	float pitch = asin(pos.y);
	float yaw = atan(pos.x, pos.z); //yaw starts from z

	{
		r0 = (M_PI / 2.0 - pitch) / M_PI;
		float r2 = r0;
		r2 = sin(M_PI * 180.0 / cam0_aov * r2) / 2.0;
		float yaw2 = yaw + M_PI + cam0_offset_yaw;
		u0 = cam0_horizon_r / cam_aspect_ratio * r2 * cos(yaw2) + 0.5 + cam0_offset_x;
		v0 = cam0_horizon_r * r2 * sin(yaw2) + 0.5 + cam0_offset_y;
	}
	{
		u1 = pos.x / -pos.y * 0.35 + 0.5;
		v1 = pos.z / -pos.y * 0.35 + 0.5;
	}
}
)glsl";

const char * fragment_shader = R"glsl(
#if (__VERSION__ > 120)
# define IN in
# define OUT out
# define texture2D texture
# define gl_FragColor FragColor
layout (location=0) out vec4 FragColor;
#else
# define IN varying
# define OUT varying
#endif // __VERSION
precision mediump float;
uniform mat4 unif_matrix;
uniform mat4 unif_matrix_1;
uniform sampler2D cam0_texture;
uniform sampler2D logo_texture;
uniform float color_offset;
uniform float color_factor;
uniform float overlap;
uniform float cam0_aov;

const float M_PI = 3.1415926535;

IN float r0;
IN float r1;
IN float u0;
IN float v0;
IN float u1;
IN float v1;

void main(void) {
	if (r0 < cam0_aov / 360.0 - overlap) {
		if (u0 <= 0.0 || u0 > 1.0 || v0 <= 0.0 || v0 > 1.0) {
			gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
		} else {
			vec4 fc = texture2D(cam0_texture, vec2(u0, v0));
			fc = (fc - color_offset) * color_factor;

			gl_FragColor = fc;
		}
	} else {
		gl_FragColor = texture2D(logo_texture, vec2(u1, v1));
	}
}
)glsl";
const char * sphere_vertex_shader = R"glsl(
#if (__VERSION__ > 120)
# define IN in
# define OUT out
#else
# define IN attribute
# define OUT varying
#endif // __VERSION
precision mediump float;
const int STEPNUM = 256;
const float STEPNUM_M1 = 255.0;

IN vec4 vPosition; //[0:1]
uniform float scale_x;
uniform float scale_y;
uniform float frame_aspect_ratio;

uniform mat4 unif_matrix;
uniform mat4 unif_matrix_1;
uniform sampler2D cam0_texture;
uniform sampler2D cam1_texture;
uniform float pixel_size;
uniform float cam_aspect_ratio;
//angular map params
uniform float r_2_pitch[STEPNUM];
//options start
uniform float sharpness_gain;
uniform float cam0_offset_yaw;
uniform float cam0_offset_x;
uniform float cam0_offset_y;
uniform float cam0_horizon_r;
uniform float cam0_aov;
uniform float cam1_offset_yaw;
uniform float cam1_offset_x;
uniform float cam1_offset_y;
uniform float cam1_horizon_r;
uniform float cam1_aov;
//options end

const float M_PI = 3.1415926535;
const float M_PI_DIV_2 = M_PI / 2.0;
const float M_PI_DIV_4 = M_PI / 4.0;
const float M_SQRT_2 = 1.4142135623;

OUT float r0;
OUT float r1;
OUT float u0;
OUT float v0;
OUT float u1;
OUT float v1;

void main(void) {
	vec4 position;
	position.xy = vPosition.xy * vec2(2.0, 2.0) + vec2(-1.0, -1.0); //[-1:1]
	gl_Position.xy = vec2(position.x, position.y * frame_aspect_ratio);
	gl_Position.zw = vec2(1.0, 1.0);

	float r = sqrt(position.x * position.x + position.y * position.y);
	if (r > M_SQRT_2) {
		r = M_SQRT_2;
	}
	float indexf = r / M_SQRT_2 * STEPNUM_M1;
	int index = int(indexf);
	float index_sub = indexf - float(index);
	float pitch_orig = r_2_pitch[index] * (1.0 - index_sub) + r_2_pitch[index + 1] * index_sub;
	float roll_orig = atan(position.y, position.x);
	if (r <= M_SQRT_2 && r > 1.0) {
		int roll_index = int(roll_orig / M_PI_DIV_2);
		float roll_base = float(roll_index) * M_PI_DIV_2 + (roll_orig > 0.0 ? M_PI_DIV_4 : -M_PI_DIV_4);
		float roll_diff = roll_orig - roll_base;
		float roll_gain = M_PI / (M_PI - 4.0 * acos(1.0 / r));
		roll_orig = roll_diff * roll_gain + roll_base;
	}
	position.x = sin(pitch_orig) * cos(roll_orig);
	position.y = sin(pitch_orig) * sin(roll_orig);
	position.z = cos(pitch_orig);
	position.w = 1.0;

	{
		vec4 pos = unif_matrix * position;
		float pitch = acos(pos.y);
		float yaw = atan(pos.x, pos.z); //yaw starts from z

		r0 = pitch / M_PI;
		float r2 = r0;
		r2 = sin(M_PI * 180.0 / cam0_aov * r2) / 2.0;
		float yaw2 = yaw + M_PI + cam0_offset_yaw;
		u0 = cam0_horizon_r * r2 / cam_aspect_ratio * cos(yaw2) + 0.5 + cam0_offset_x;
		v0 = cam0_horizon_r * r2 * sin(yaw2) + 0.5 + cam0_offset_y;
	}
	{
		vec4 pos = unif_matrix_1 * position;
		float pitch = acos(pos.y);
		float yaw = atan(pos.x, pos.z);

		r1 = pitch / M_PI;
		float r2 = 1.0 - r1;
		r2 = sin(M_PI * 180.0 / cam1_aov * r2) / 2.0;
		float yaw2 = -yaw + M_PI + cam1_offset_yaw;
		u1 = cam1_horizon_r * r2 / cam_aspect_ratio * cos(yaw2) + 0.5 + cam1_offset_x;
		v1 = cam1_horizon_r * r2 * sin(yaw2) + 0.5 + cam1_offset_y;
	}
}
)glsl";
const char * sphere_fragment_shader = R"glsl(
#if (__VERSION__ > 120)
# define IN in
# define OUT out
# define texture2D texture
# define gl_FragColor FragColor
layout (location=0) out vec4 FragColor;
#else
# define IN varying
# define OUT varying
#endif // __VERSION
precision mediump float;
uniform mat4 unif_matrix;
uniform mat4 unif_matrix_1;
uniform sampler2D cam0_texture;
uniform sampler2D cam1_texture;
uniform float color_offset;
uniform float color_factor;
uniform float overlap;

const float M_PI = 3.1415926535;

OUT float r0;
OUT float r1;
OUT float u0;
OUT float v0;
OUT float u1;
OUT float v1;

void main(void) {
	vec4 fc0;
	vec4 fc1;
	if (r0 < 0.5 + overlap) {
		if (u0 <= 0.0 || u0 > 1.0 || v0 <= 0.0 || v0 > 1.0) {
			fc0 = vec4(0.0, 0.0, 0.0, 1.0);
		} else {
			vec4 fc = texture2D(cam0_texture, vec2(u0, v0));
			fc = (fc - color_offset) * color_factor;

			fc0 = fc;
		}
	}

	if (r1 > 0.5 - overlap) {
		if (u1 <= 0.0 || u1 > 1.0 || v1 <= 0.0 || v1 > 1.0) {
			fc1 = vec4(0.0, 0.0, 0.0, 1.0);
		} else {
			vec4 fc = texture2D(cam1_texture, vec2(u1, v1));
			fc = (fc - color_offset) * color_factor;

			fc1 = fc;
		}
	}
	if (r0 < 0.5 - overlap) {
		gl_FragColor = fc0;
	} else if (r0 < 0.5 + overlap) {
		gl_FragColor = (fc0 * ((0.5 + overlap) - r0) + fc1 * (r0 - (0.5 - overlap))) / (overlap * 2.0);
	} else {
		gl_FragColor = fc1;
	}
}
)glsl";
static void init(void *obj, const char *common, int num_of_cam) {
	picam360map_renderer *_this = (picam360map_renderer*) obj;

	_this->num_of_cam = num_of_cam;

	board_mesh(64, &_this->vbo, &_this->vbo_nop, &_this->vao);
	if (_this->num_of_cam == 1) {
		_this->program = GLProgram_new(common, vertex_shader, fragment_shader, false);
	} else {
		_this->program = GLProgram_new(common, sphere_vertex_shader, sphere_fragment_shader, false);
	}
}
static void release(void *obj) {
	picam360map_renderer *_this = (picam360map_renderer*) obj;
	int status;
	free(obj);
}
static int get_program(void *obj) {
	picam360map_renderer *_this = (picam360map_renderer*) obj;
	return GLProgram_GetId(_this->program);
}
static void render(void *obj) {
	picam360map_renderer *_this = (picam360map_renderer*) obj;

	glBindBuffer(GL_ARRAY_BUFFER, _this->vbo);

#ifdef USE_GLES
	GLuint loc = glGetAttribLocation(program, "vPosition");
	glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(loc);
#else
	glBindVertexArray(model->vao);
#endif

	glDrawArrays(GL_TRIANGLE_STRIP, 0, _this->vbo_nop);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
#ifdef USE_GLES
	glDisableVertexAttribArray(loc);
#else
	glBindVertexArray(0);
#endif
}

static int board_mesh(int num_of_steps, GLuint *vbo_out, GLuint *n_out, GLuint *vao_out) {
	GLuint vbo;

	int n = 2 * (num_of_steps + 1) * num_of_steps;
	float points[4 * n];

	float start_x = 0.0f;
	float start_y = 0.0f;

	float end_x = 1.0f;
	float end_y = 1.0f;

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
				points[idx++] = x;
				points[idx++] = y;
				points[idx++] = z;
				points[idx++] = 1.0;
				//printf("x=%f,y=%f,z=%f,w=%f\n", points[idx - 4],
				//		points[idx - 3], points[idx - 2], points[idx - 1]);
			}
			{
				float x = start_x + step_x * (i + 1);
				float y = start_y + step_y * j;
				float z = 1.0;
				points[idx++] = x;
				points[idx++] = y;
				points[idx++] = z;
				points[idx++] = 1.0;
				//printf("x=%f,y=%f,z=%f,w=%f\n", points[idx - 4],
				//		points[idx - 3], points[idx - 2], points[idx - 1]);
			}
		}
	}

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * n, points, GL_STATIC_DRAW);

#ifdef USE_GLES
#else
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

static void create_renderer(void *user_data, RENDERER_T **out_renderer) {
	RENDERER_T *renderer = (RENDERER_T*) malloc(sizeof(picam360map_renderer));
	memset(renderer, 0, sizeof(picam360map_renderer));
	strcpy(renderer->name, RENDERER_NAME);
	renderer->release = release;
	renderer->init = init;
	renderer->get_program = get_program;
	renderer->render = render;
	renderer->user_data = renderer;

	if (out_renderer) {
		*out_renderer = renderer;
	}
}

static int command_handler(void *user_data, const char *_buff) {
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
}

static void init_options(void *user_data, json_t *options) {
}

static void save_options(void *user_data, json_t *options) {
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
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
		RENDERER_T *renderer = NULL;
		create_renderer(NULL, &renderer);
		lg_plugin_host->add_renderer(renderer);
	}
}
