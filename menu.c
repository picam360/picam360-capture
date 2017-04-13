#include "menu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdbool.h>
#include <linux/input.h>
#include <fcntl.h>
#include <wchar.h>

#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"

#include "texture-atlas.h"
#include "texture-font.h"
#include "gl_program.h"

typedef struct {
	void *program;
	GLuint vbo;
	GLuint vbo_nop;
} MODEL_T;
struct {
	texture_font_t *font;
	texture_atlas_t *atlas;
	MODEL_T model;
} lg_freetypegles;

// --------------------------------------------------------------- add_text ---
void add_text(vector_t * vVector, texture_font_t * font, wchar_t * text,
		vec4 * color, vec2 * pen) {
	size_t i;
	float r = color->red, g = color->green, b = color->blue, a = color->alpha;
	for (i = 0; i < wcslen(text); ++i) {
		texture_glyph_t *glyph = texture_font_get_glyph(font, text[i]);
		if (glyph != NULL) {
			int kerning = 0;
			if (i > 0) {
				kerning = texture_glyph_get_kerning(glyph, text[i - 1]);
			}
			pen->x += kerning;
			int x0 = (int) (pen->x + glyph->offset_x);
			int y0 = (int) (pen->y + glyph->offset_y);
			int x1 = (int) (x0 + glyph->width);
			int y1 = (int) (y0 - glyph->height);
			float s0 = glyph->s0;
			float t0 = glyph->t0;
			float s1 = glyph->s1;
			float t1 = glyph->t1;

			// data is x,y,z,s,t,r,g,b,a
			GLfloat vertices[] = { x0, y0, 0, s0, t0, r, g, b, a, x0, y1, 0, s0,
					t1, r, g, b, a, x1, y1, 0, s1, t1, r, g, b, a, x0, y0, 0,
					s0, t0, r, g, b, a, x1, y1, 0, s1, t1, r, g, b, a, x1, y0,
					0, s1, t0, r, g, b, a };

			vector_push_back_data(vVector, vertices, 9 * 6);

			pen->x += glyph->advance_x;
		}
	}
}

void init_menu() {
	// all the shaders have at least texture unit 0 active so
	// activate it now and leave it active
	glActiveTexture(GL_TEXTURE0);

	/* Texture atlas to store individual glyphs */
	lg_freetypegles.atlas = texture_atlas_new(1024, 1024, 1);

	lg_freetypegles.font = texture_font_new(lg_freetypegles.atlas,
			"./libs/freetypeGlesRpi/fonts/custom.ttf", 10);

	/* Cache some glyphs to speed things up */
	texture_font_load_glyphs(lg_freetypegles.font,
			L" !\"#$%&'()*+,-./0123456789:;<=>?"
					L"@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_"
					L"`abcdefghijklmnopqrstuvwxyz{|}~");

	lg_freetypegles.model.program = GLProgram_new("shader/freetypegles.vert",
			"shader/freetypegles.frag");

	texture_atlas_upload(lg_freetypegles.atlas);
}
void deinit_menu() {
	//todo
}

MENU_T *menu_new(char *name, MENU_CALLBACK callback){
	MENU_T *menu = (MENU_T*)malloc(sizeof(MENU_T));
	strncpy(menu->name, name, sizeof(menu->name));
	menu->callback = callback;
	return menu;
}

void menu_delete(MENU_T **menu){
	free(*menu);
	*menu = NULL;
}

void menu_redraw(MENU_T *root, wchar_t *_status, uint32_t screen_width,
		uint32_t screen_height, uint32_t frame_width, uint32_t frame_height,
		bool stereo) {
	int program = GLProgram_GetId(lg_freetypegles.model.program);
	glUseProgram(program);

	const int MAX_STATUS_LEN = 1024;
	wchar_t status[1024];
	wcsncpy(status, _status, MAX_STATUS_LEN - 1);
	status[MAX_STATUS_LEN - 1] = L'\0';	//fail safe

	vector_t * vVector = vector_new(sizeof(GLfloat));

	vec2 pen = { };
	vec4 color = { 1, 1, 1, 1 };
	vec4 back_color = { 0.2, 0.2, 0.2, 1 };

	int line = 0;
	{
		wchar_t *ptr;
		wchar_t *tok = wcstok(status, L"\n", &ptr);
		while (tok) {
			pen.x =
					-((float) screen_width / 2 - lg_freetypegles.font->size / 8);
			pen.y = ((float) screen_height / 2 - lg_freetypegles.font->size / 8)
					- lg_freetypegles.font->size * (line + 1);
			add_text(vVector, lg_freetypegles.font, tok, &back_color, &pen);

			pen.x = -((float) screen_width / 2);
			pen.y = ((float) screen_height / 2)
					- lg_freetypegles.font->size * (line + 1);
			add_text(vVector, lg_freetypegles.font, tok, &color, &pen);

			line++;
			tok = wcstok(NULL, L"\n", &ptr);
		}
	}

	// Use the program object
	glUseProgram(program);

	int vertexHandle, texHandle, samplerHandle, colorHandle, mvpHandle;
	// Bind vPosition to attribute 0
	vertexHandle = glGetAttribLocation(program, "a_position");
	texHandle = glGetAttribLocation(program, "a_st");
	colorHandle = glGetAttribLocation(program, "a_color");
	samplerHandle = glGetUniformLocation(program, "texture_uniform");

	mvpHandle = glGetUniformLocation(program, "u_mvp");

	float a = 1.0f / (screen_width / 2);
	float b = 1.0f / (screen_height / 2);

	GLfloat mvp[] = { //
			//
					a, 0, 0, 0,		//
					0, b, 0, 0,		//
					0, 0, 1.0, 0,		//
					0, 0, 0, 1.0		//
			};
	glUniformMatrix4fv(mvpHandle, 1, GL_FALSE, (GLfloat *) mvp);

// Load the vertex data
	glVertexAttribPointer(vertexHandle, 3, GL_FLOAT, GL_FALSE,
			9 * sizeof(GLfloat), vVector->items);
	glEnableVertexAttribArray(vertexHandle);
	glVertexAttribPointer(texHandle, 2, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat),
			(GLfloat*) vVector->items + 3);
	glEnableVertexAttribArray(texHandle);
	glVertexAttribPointer(colorHandle, 4, GL_FLOAT, GL_FALSE,
			9 * sizeof(GLfloat), (GLfloat*) vVector->items + 5);
	glEnableVertexAttribArray(colorHandle);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, lg_freetypegles.atlas->id);

	glUniform1i(samplerHandle, 0);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	if (stereo) {
		int offset_x = (screen_width / 2 - frame_width) / 2;
		int offset_y = (screen_height - frame_height) / 2;
		for (int i = 0; i < 2; i++) {
			glViewport(offset_x + i * screen_width / 2, offset_y,
					(GLsizei) frame_width, (GLsizei) frame_height);
			glDrawArrays(GL_TRIANGLES, 0, vVector->size / 9);
		}
	} else {
		int offset_x = (screen_width - frame_width) / 2;
		int offset_y = (screen_height - frame_height) / 2;
		glViewport(offset_x, offset_y, (GLsizei) frame_width,
				(GLsizei) frame_height);
		glDrawArrays(GL_TRIANGLES, 0, vVector->size / 9);
	}

	glDisableVertexAttribArray(vertexHandle);
	glDisableVertexAttribArray(texHandle);
	glDisableVertexAttribArray(colorHandle);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);

	vector_delete(vVector);
}
void menu_add_submenu(MENU_T *parent, MENU_T *child, int idx){

}
void menu_operate(MENU_T *menu, enum MENU_OPERATE operate){

}
