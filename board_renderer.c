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
#include "glsl/board_fsh.h"
#include "glsl/board_vsh.h"

#define RENDERER_NAME "BOARD"

typedef struct _board_renderer {
	RENDERER_T super;

	int num_of_cam;
	void *program_obj;
	GLuint vbo;
	GLuint vbo_nop;
	GLuint vao;

	void *user_data;
} board_renderer;

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
	board_renderer *_this = (board_renderer*) obj;

	_this->num_of_cam = num_of_cam;

	board_mesh(1, &_this->vbo, &_this->vbo_nop, &_this->vao);
	{
		const char *fsh_filepath = "/tmp/tmp.fsh";
		const char *vsh_filepath = "/tmp/tmp.vsh";
		int fsh_fd = open(fsh_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		int vsh_fd = open(vsh_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		write(fsh_fd, board_fsh, board_fsh_len);
		write(vsh_fd, board_vsh, board_vsh_len);
		close(fsh_fd);
		close(vsh_fd);
		_this->program_obj = GLProgram_new(common, vsh_filepath, fsh_filepath, true);
		remove(fsh_filepath);
		remove(vsh_filepath);
	}
}
static void release(void *obj) {
	board_renderer *_this = (board_renderer*) obj;
	int status;
	free(obj);
}
static int get_program(void *obj) {
	board_renderer *_this = (board_renderer*) obj;
	return GLProgram_GetId(_this->program_obj);
}

static void render(void *obj, float fov) {
	board_renderer *_this = (board_renderer*) obj;

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

void create_board_renderer(PLUGIN_HOST_T *plugin_host, RENDERER_T **out_renderer) {
	RENDERER_T *renderer = (RENDERER_T*) malloc(sizeof(board_renderer));
	memset(renderer, 0, sizeof(board_renderer));
	strcpy(renderer->name, RENDERER_NAME);
	renderer->release = release;
	renderer->init = init;
	renderer->get_program = get_program;
	renderer->render = render;
	renderer->user_data = renderer;

	board_renderer *_this = (board_renderer*) renderer;

	if (out_renderer) {
		*out_renderer = renderer;
	}
}
