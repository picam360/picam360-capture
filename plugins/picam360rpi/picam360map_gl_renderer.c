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

#include "picam360map_renderer.h"
#include "glsl/picam360map_fsh.h"
#include "glsl/picam360map_vsh.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PLUGIN_NAME "picam360map_renderer"
#define RENDERER_NAME "PICAM360MAP"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

//rtp or uvc
static char lg_options_input_type[32] = { 'u', 'v', 'c' };
static char lg_options_input_codec[32] = { 'm', 'j', 'p', 'e', 'g' };

typedef struct _picam360map_renderer {
	RENDERER_T super;

	int num_of_cam;
	void *program_obj;
	GLuint vbo;
	GLuint vbo_nop;
	GLuint vao;

	void *user_data;
} picam360map_renderer;

#ifdef REC709
// ITU-T Rec. 709
static float YUV2RGB[] = { //
	1.16438, 0.00000, 1.79274, -0.97295,//
	1.16438, -0.21325, -0.53291, 0.30148,//
	1.16438, 2.11240, 0.00000, -1.13340,//
	0, 0, 0, 1,//
};
#else
// assume ITU-T Rec. 601
static float YUV2RGB[] = { //
		1.16438, 0.00000, 1.59603, -0.87079, //
				1.16438, -0.39176, -0.81297, 0.52959, //
				1.16438, 2.01723, 0.00000, -1.08139, //
				0, 0, 0, 1 //
		};
#endif

/*
 int spherewindow_mesh(int num_of_steps, GLuint *vbo_out, GLuint *n_out, GLuint *vao_out) {
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
 float theta = i * 2 * M_PI / num_of_steps;
 float phi = j * M_PI / num_of_steps;
 float r = sin(phi);	//phi starts from y axis
 float x = r * cos(theta);
 float y = cos(phi);	//phi starts from y axis
 float z = r * sin(theta);
 points[idx++] = x;
 points[idx++] = y;
 points[idx++] = z;
 points[idx++] = 1.0;
 //printf("x=%f,y=%f,z=%f,w=%f\n", points[idx - 4],
 //		points[idx - 3], points[idx - 2], points[idx - 1]);
 }
 {
 float theta = (i + 1) * 2 * M_PI / num_of_steps;
 float phi = j * M_PI / num_of_steps;
 float r = sin(phi);	//phi starts from y axis
 float x = r * cos(theta);
 float y = cos(phi);	//phi starts from y axis
 float z = r * sin(theta);
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
 */

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
	picam360map_renderer *_this = (picam360map_renderer*) obj;

	_this->num_of_cam = num_of_cam;

	board_mesh(64, &_this->vbo, &_this->vbo_nop, &_this->vao);
	{
		const char *tmp_fsh_filepath = "/tmp/tmptmp.fsh";
		const char *tmp_vsh_filepath = "/tmp/tmptmp.vsh";
		const char *fsh_filepath = "/tmp/tmp.fsh";
		const char *vsh_filepath = "/tmp/tmp.vsh";
		int fsh_fd = open(tmp_fsh_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		int vsh_fd = open(tmp_vsh_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);

		write(fsh_fd, picam360map_fsh, picam360map_fsh_len);
		write(vsh_fd, picam360map_vsh, picam360map_vsh_len);
		close(fsh_fd);
		close(vsh_fd);
		{//fsh
			int buff_cur = 0;
			char buff[1024] = {};
			buff_cur += sprintf(buff + buff_cur, "sed");
			buff_cur += sprintf(buff + buff_cur, " -e 's/%%NUM_OF_CAM%%/%d/g'", MAX(1, num_of_cam));
			{
				int buff2_cur = 0;
				char buff2[1024] = {};
				for (int i = 0; i < num_of_cam; i++) {
					buff2_cur += sprintf(buff2 + buff2_cur, "FCS(%d);", i);
				}
				buff_cur += sprintf(buff + buff_cur, " -e 's/%%FOR_FCS_NUM_OF_CAM%%/%s/g'", buff2);
			}
			buff_cur += sprintf(buff + buff_cur, " %s > %s", tmp_fsh_filepath, fsh_filepath);
			system(buff);
		}
		{//vsh
			int buff_cur = 0;
			char buff[1024] = {};
			buff_cur += sprintf(buff + buff_cur, "sed");
			buff_cur += sprintf(buff + buff_cur, " -e 's/%%NUM_OF_CAM%%/%d/g'", MAX(1, num_of_cam));
			{
				int buff2_cur = 0;
				char buff2[1024] = {};
				for (int i = 0; i < num_of_cam; i++) {
					buff2_cur += sprintf(buff2 + buff2_cur, "CAM_UVR(%d);", i);
				}
				buff_cur += sprintf(buff + buff_cur, " -e 's/%%FOR_CAM_UVR_NUM_OF_CAM%%/%s/g'", buff2);
			}
			buff_cur += sprintf(buff + buff_cur, " %s > %s", tmp_vsh_filepath, vsh_filepath);
			system(buff);
		}
		_this->program_obj = GLProgram_new(common, vsh_filepath, fsh_filepath, true);
		remove(tmp_fsh_filepath);
		remove(tmp_vsh_filepath);
		remove(fsh_filepath);
		remove(vsh_filepath);
	}
}
static void release(void *obj) {
	picam360map_renderer *_this = (picam360map_renderer*) obj;
	int status;
	free(obj);
}
static int get_program(void *obj) {
	picam360map_renderer *_this = (picam360map_renderer*) obj;
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
	picam360map_renderer *_this = (picam360map_renderer*) obj;

	int program = GLProgram_GetId(_this->program_obj);
	glUniformMatrix4fv(glGetUniformLocation(program, "YUV2RGB"), 1, GL_FALSE, (GLfloat*) YUV2RGB);
	{
		const int stepnum = 256;
		float fov_min = 30;
		float fov_max = 120;
		float fov_factor = 1.0 - (MIN(MAX(fov / 2.0, fov_min), fov_max) - fov_min) / (fov_max - fov_min) / 2.0;
		//float ganma = log(frame->fov / 2.0 / 180.0) / log(1.0 / sqrt(2));
		//float x_ary[3] = { 0.0, 1.0, sqrt(2) };
		//float y_ary[3] = { 0.0, frame->fov / 1.2 * M_PI / 180.0, M_PI };
		float x_ary2[stepnum];
		float y_ary2[stepnum];
		POINT_T p0 = { 0.0, 0.0 };
		POINT_T p1 = { fov_factor, -fov_factor + 1.0 };
		POINT_T p2 = { 1.0, 1.0 };
		for (int i = 0; i < stepnum; i++) {
			POINT_T p = QuadraticBezPoint(p0, p1, p2, (float) i / (stepnum - 1));
			x_ary2[i] = p.x;
			y_ary2[i] = p.y;
		}
		// invert x y
		const int stepnum3 = 256;
		float x_ary3[stepnum3];
		float y_ary3[stepnum3];
		for (int i = 0; i < stepnum3; i++) {
			x_ary3[i] = (float) i / (stepnum3 - 1);
			for (int j = 0; j < stepnum - 1; j++) {
				if (x_ary3[i] >= x_ary2[j] && x_ary3[i] <= x_ary2[j + 1]) {
					float ratio = (x_ary3[i] - x_ary2[j]) / (x_ary2[j + 1] - x_ary2[j]);
					y_ary3[i] = ratio * (y_ary2[j + 1] - y_ary2[j]) + y_ary2[j];
					y_ary3[i] *= M_PI;
					break;
				}
			}
		}
		glUniform1fv(glGetUniformLocation(program, "r_2_pitch"), stepnum3, y_ary3);
		//		static bool init = false;
		//		if (!init) {
		//			init = true;
		//			for (int i = 0; i < stepnum; i++) {
		//				printf("%f,%f\n", x_ary2[i], y_ary2[i]);
		//			}
		//		}
	}
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
