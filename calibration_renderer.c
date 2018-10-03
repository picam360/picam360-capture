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
#include "picam360_capture_plugin.h"

#define RENDERER_NAME "CALIBRATION"

typedef struct _calibration_renderer {
	RENDERER_T super;

	int num_of_cam;
	void *program_obj;
	GLuint vbo;
	GLuint vbo_nop;
	GLuint vao;
	uint32_t calibration_texture;

	void *user_data;
} calibration_renderer;

const char * vertex_shader = R"glsl(
#if (__VERSION__ > 120)
# define IN in
# define OUT out
#else
# define IN attribute
# define OUT varying
#endif // __VERSION
precision mediump float;
IN vec4 vPosition;
OUT vec2 tcoord;

void main(void) {
	vec4 pos = vPosition;
	tcoord = pos.xy;
	pos.xy = pos.xy * vec2(2, 2) + vec2(-1, -1);
	gl_Position = pos;
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

const int MAX_NUM_OF_CAM =
		3;
IN vec2 tcoord;
uniform sampler2D logo_texture;
uniform sampler2D cam_texture[MAX_NUM_OF_CAM];
uniform float cam_offset_x[MAX_NUM_OF_CAM];
uniform float cam_offset_y[MAX_NUM_OF_CAM];
uniform float cam_horizon_r[MAX_NUM_OF_CAM];
uniform float pixel_size;
uniform int active_cam;
uniform float sharpness_gain;
uniform float cam_aspect_ratio;

void main(void) {
	vec4 fc = texture2D(logo_texture, vec2(tcoord.x, tcoord.y));
	if (fc.g == 0.0) {
	)glsl"
#ifdef TEGRA
	R"glsl(
		float u = (tcoord.x + cam_offset_x[active_cam] - 0.5)/cam_aspect_ratio + 0.5;
		float v = tcoord.y + get_cam_offset_y[active_cam];
		if (sharpness_gain == 0.0) {
			fc = texture2D(cam_texture[active_cam], vec2(u, v));
		} else {
			//sharpness
			float gain = sharpness_gain;
			fc = texture2D(cam_texture[active_cam], vec2(u, v))
					* (1.0 + 4.0 * gain);
			fc -= texture2D(cam_texture[active_cam], vec2(u - 1.0 * pixel_size, v))
					* gain;
			fc -= texture2D(cam_texture[active_cam], vec2(u, v - 1.0 * pixel_size))
					* gain;
			fc -= texture2D(cam_texture[active_cam], vec2(u, v + 1.0 * pixel_size))
					* gain;
			fc -= texture2D(cam_texture[active_cam], vec2(u + 1.0 * pixel_size, v))
					* gain;
		}
)glsl"
#else
R"glsl(
		if(active_cam == 0) {
			const int i = 0;
			float u = (tcoord.x + cam_offset_x[i] - 0.5)/cam_aspect_ratio + 0.5;
			float v = tcoord.y + cam_offset_y[i];
			if (sharpness_gain == 0.0) {
				fc = texture2D(cam_texture[i], vec2(u, v));
			} else {
				//sharpness
				float gain = sharpness_gain;
				fc = texture2D(cam_texture[i], vec2(u, v))
				* (1.0 + 4.0 * gain);
				fc -= texture2D(cam_texture[i], vec2(u - 1.0 * pixel_size, v))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u, v - 1.0 * pixel_size))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u, v + 1.0 * pixel_size))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u + 1.0 * pixel_size, v))
				* gain;
			}
		} else if(active_cam == 1) {
			const int i = 1;
			float u = (tcoord.x + cam_offset_x[i] - 0.5)/cam_aspect_ratio + 0.5;
			float v = tcoord.y + cam_offset_y[i];
			if (sharpness_gain == 0.0) {
				fc = texture2D(cam_texture[i], vec2(u, v));
			} else {
				//sharpness
				float gain = sharpness_gain;
				fc = texture2D(cam_texture[i], vec2(u, v))
				* (1.0 + 4.0 * gain);
				fc -= texture2D(cam_texture[i], vec2(u - 1.0 * pixel_size, v))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u, v - 1.0 * pixel_size))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u, v + 1.0 * pixel_size))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u + 1.0 * pixel_size, v))
				* gain;
			}
		}
	)glsl"
#endif
	R"glsl(
	}
	gl_FragColor = fc;
}
)glsl";

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

static void init(void *obj, const char *common, int num_of_cam) {
	calibration_renderer *_this = (calibration_renderer*) obj;

	_this->num_of_cam = num_of_cam;

	board_mesh(1, &_this->vbo, &_this->vbo_nop, &_this->vao);
	_this->program_obj = GLProgram_new(common, vertex_shader, fragment_shader, false);
}
static void release(void *obj) {
	calibration_renderer *_this = (calibration_renderer*) obj;
	int status;
	free(obj);
}
static int get_program(void *obj) {
	calibration_renderer *_this = (calibration_renderer*) obj;
	return GLProgram_GetId(_this->program_obj);
}

typedef struct _POINT_T {
	float x;
	float y;
} POINT_T;

static POINT_T QuadraticBezPoint(POINT_T p0, POINT_T p1, POINT_T p2, float d) {

	POINT_T o = { 0, 0 };

	float v = (1 - d) * (1 - d);
	o.x += v * p0.x;
	o.y += v * p0.y;

	v = 2 * d * (1 - d);
	o.x += v * p1.x;
	o.y += v * p1.y;

	v = d * d;
	o.x += v * p2.x;
	o.y += v * p2.y;

	return o;
}

static void render(void *obj, float fov) {
	calibration_renderer *_this = (calibration_renderer*) obj;

	int program = GLProgram_GetId(_this->program_obj);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, _this->calibration_texture);

	glBindBuffer(GL_ARRAY_BUFFER, _this->vbo);

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

void create_calibration_renderer(PLUGIN_HOST_T *plugin_host, RENDERER_T **out_renderer) {
	RENDERER_T *renderer = (RENDERER_T*) malloc(sizeof(calibration_renderer));
	memset(renderer, 0, sizeof(calibration_renderer));
	strcpy(renderer->name, RENDERER_NAME);
	renderer->release = release;
	renderer->init = init;
	renderer->get_program = get_program;
	renderer->render = render;
	renderer->user_data = renderer;

	calibration_renderer *_this = (calibration_renderer*) renderer;
	plugin_host->load_texture("img/calibration_img.png", &_this->calibration_texture);

	if (out_renderer) {
		*out_renderer = renderer;
	}
}
