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
#include "glsl/equirectangular_fsh.h"
#include "glsl/equirectangular_vsh.h"

#include "equirectangular_renderer.h"

#define PLUGIN_NAME "equirectangular_renderer"
#define RENDERER_NAME "EQUIRECTANGULAR"

static PLUGIN_HOST_T *lg_plugin_host = NULL;

//rtp or uvc
static char lg_options_input_type[32] = { 'u', 'v', 'c' };
static char lg_options_input_codec[32] = { 'm', 'j', 'p', 'e', 'g' };

typedef struct _equirectangular_renderer {
	RENDERER_T super;

	int num_of_cam;
	void *program_obj;
	GLuint vbo;
	GLuint vbo_nop;
	GLuint vao;

	void *user_data;
} equirectangular_renderer;

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
	equirectangular_renderer *_this = (equirectangular_renderer*) obj;

	_this->num_of_cam = num_of_cam;

	board_mesh(64, &_this->vbo, &_this->vbo_nop, &_this->vao);
	{
		const char *fsh_filepath = "/tmp/tmp.fsh";
		const char *vsh_filepath = "/tmp/tmp.vsh";
		int fsh_fd = open(fsh_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		int vsh_fd = open(vsh_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		write(fsh_fd, equirectangular_fsh, equirectangular_fsh_len);
		write(vsh_fd, equirectangular_vsh, equirectangular_vsh_len);
		close(fsh_fd);
		close(vsh_fd);
		_this->program_obj = GLProgram_new(common, vsh_filepath, fsh_filepath, true);
		remove(fsh_filepath);
		remove(vsh_filepath);
	}
}
static void release(void *obj) {
	equirectangular_renderer *_this = (equirectangular_renderer*) obj;
	int status;
	free(obj);
}
static int get_program(void *obj) {
	equirectangular_renderer *_this = (equirectangular_renderer*) obj;
	return GLProgram_GetId(_this->program_obj);
}
static void render(void *obj, float fov) {
	equirectangular_renderer *_this = (equirectangular_renderer*) obj;

	int program = GLProgram_GetId(_this->program_obj);

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
	RENDERER_T *renderer = (RENDERER_T*) malloc(sizeof(equirectangular_renderer));
	memset(renderer, 0, sizeof(equirectangular_renderer));
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
