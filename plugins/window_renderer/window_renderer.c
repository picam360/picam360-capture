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

#include "window_renderer.h"

#define PLUGIN_NAME "window_renderer"
#define RENDERER_NAME "WINDOW"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

typedef struct _window_renderer {
	RENDERER_T super;

	int num_of_cam;
	void *program_obj;
	GLuint vbo;
	GLuint vbo_nop;
	GLuint vao;

	void *user_data;
} window_renderer;

static const char * vertex_shader = R"glsl(
#if (__VERSION__ > 120)
# define IN in
# define OUT out
#else
# define IN attribute
# define OUT varying
#endif // __VERSION
precision highp float;

const int MAX_NUM_OF_CAM =
		3;
const float M_PI = 3.1415926535;
const float M_PI_DIV_2 = M_PI / 2.0;
const float M_PI_DIV_4 = M_PI / 4.0;
const float M_SQRT_2 = 1.4142135623;

IN vec4 vPosition; //[0:1]
//options start
uniform float scale;
uniform float frame_aspect_ratio;

uniform float pixel_size;
uniform float cam_aspect_ratio;
uniform float sharpness_gain;

uniform int num_of_cam;
uniform sampler2D cam_texture[MAX_NUM_OF_CAM];
uniform mat4 cam_attitude[MAX_NUM_OF_CAM];
uniform float cam_offset_x[MAX_NUM_OF_CAM];
uniform float cam_offset_y[MAX_NUM_OF_CAM];
uniform float cam_horizon_r[MAX_NUM_OF_CAM];
uniform float cam_aov[MAX_NUM_OF_CAM];
//options end

OUT vec3 cam_uvr[MAX_NUM_OF_CAM];
OUT vec3 logo_uvr;

void main(void) {
	vec4 position = vPosition;
	gl_Position = vec4(vPosition.x / vPosition.z * scale, vPosition.y / vPosition.z * scale * frame_aspect_ratio, 1.0, 1.0);

	//if(num_of_cam >= 1)
	{ //cam0
		const int i = 0;
		vec4 pos = cam_attitude[i] * position;
		float pitch = asin(pos.y);
		float yaw = atan(pos.x, pos.z);

		float r = (M_PI / 2.0 - pitch) / M_PI;
		float r2 = sin(M_PI * 180.0 / cam_aov[i] * r) / 2.0;
		cam_uvr[i][0] = cam_horizon_r[i] / cam_aspect_ratio * r2 * cos(yaw) + 0.5 + cam_offset_x[i];
		cam_uvr[i][1] = cam_horizon_r[i] * r2 * sin(yaw) + 0.5 + cam_offset_y[i];
		cam_uvr[i][2] = r;
	}
	{ //logo
		vec4 pos = cam_attitude[0] * position;
		logo_uvr[0] = pos.x / -pos.y * 0.35 + 0.5;
		logo_uvr[1] = pos.z / -pos.y * 0.35 + 0.5;
	}
}
)glsl";

static const char * fragment_shader = R"glsl(
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
precision highp float;

const int MAX_NUM_OF_CAM =
		3;
const float M_PI = 3.1415926535;

uniform int num_of_cam;
uniform sampler2D cam_texture[MAX_NUM_OF_CAM];
uniform float cam_aov[MAX_NUM_OF_CAM];
uniform sampler2D logo_texture;
uniform float color_offset;
uniform float color_factor;
uniform float overlap;

IN vec3 cam_uvr[MAX_NUM_OF_CAM];
IN vec3 logo_uvr;

void main(void) {
	vec4 fcs[MAX_NUM_OF_CAM];
	float alpha = 0.0;
	//if(num_of_cam >= 1)
	{
		const int i = 0;
		float r_thresh = cam_aov[i] / 360.0;
		if (cam_uvr[i][2] > r_thresh || cam_uvr[i][0] <= 0.0 || cam_uvr[i][0] > 1.0 || cam_uvr[i][1] <= 0.0 || cam_uvr[i][1] > 1.0) {
			fcs[i] = vec4(0.0, 0.0, 0.0, 0.0);
		} else {
			fcs[i] = texture2D(cam_texture[i], vec2(cam_uvr[i][0], cam_uvr[i][1]));
			fcs[i].a = 1.0 - cam_uvr[i][2] / r_thresh;
			alpha += fcs[i].a;
		}
	}
	if (num_of_cam >= 2) {
		const int i = 1;
		float r_thresh = cam_aov[i] / 360.0;
		if (cam_uvr[i][2] > r_thresh || cam_uvr[i][0] <= 0.0 || cam_uvr[i][0] > 1.0 || cam_uvr[i][1] <= 0.0 || cam_uvr[i][1] > 1.0) {
			fcs[i] = vec4(0.0, 0.0, 0.0, 0.0);
		} else {
			fcs[i] = texture2D(cam_texture[i], vec2(cam_uvr[i][0], cam_uvr[i][1]));
			fcs[i].a = 1.0 - cam_uvr[i][2] / r_thresh;
			alpha += fcs[i].a;
		}
	}
	if (alpha == 0.0) {
		gl_FragColor = texture2D(logo_texture, vec2(logo_uvr.x, logo_uvr.y));
	} else {
		vec4 fc = vec4(0.0, 0.0, 0.0, 0.0);
		//if(num_of_cam >= 1)
		{
			const int i = 0;
			fc += fcs[i] * (fcs[i].a / alpha);
		}
		if (num_of_cam >= 2) {
			const int i = 1;
			fc += fcs[i] * (fcs[i].a / alpha);
		}
		fc = (fc - color_offset) * color_factor;
		gl_FragColor = fc;
	}
}
)glsl";

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

	if (vbo_out != NULL)
	*vbo_out = vbo;
	if (n_out != NULL)
	*n_out = n;

	return 0;
}

static void init(void *obj, const char *common, int num_of_cam) {
	window_renderer *_this = (window_renderer*) obj;

	_this->num_of_cam = num_of_cam;

	float maxfov = 150.0;
	spherewindow_mesh(maxfov, maxfov, 64, &_this->vbo, &_this->vbo_nop, &_this->vao);
	_this->program_obj = GLProgram_new(common, vertex_shader, fragment_shader, false);
}
static void release(void *obj) {
	window_renderer *_this = (window_renderer*) obj;
	int status;
	free(obj);
}
static int get_program(void *obj) {
	window_renderer *_this = (window_renderer*) obj;
	return GLProgram_GetId(_this->program_obj);
}
static void render(void *obj, float fov) {
	window_renderer *_this = (window_renderer*) obj;

	int program = GLProgram_GetId(_this->program_obj);

	glBindBuffer(GL_ARRAY_BUFFER, _this->vbo);

	float fov_rad = fov * M_PI / 180.0;
	float scale = 1.0 / tan(fov_rad / 2);
	glUniform1f(glGetUniformLocation(program, "scale"), scale);

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

static void create_renderer(void *user_data, RENDERER_T **out_renderer) {
	RENDERER_T *renderer = (RENDERER_T*) malloc(sizeof(window_renderer));
	memset(renderer, 0, sizeof(window_renderer));
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
