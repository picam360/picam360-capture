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
#include <fcntl.h>
#include <wchar.h>
#include <limits.h>

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

#include "texture-atlas.h"
#include "texture-font.h"
#include "gl_program.h"
#include "glsl/freetype_fsh.h"
#include "glsl/freetype_vsh.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

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
void add_text(vector_t * vVector, texture_font_t * font, wchar_t * text, vec4 * color, vec2 * pen) {
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
			GLfloat vertices[] = { x0, y0, 0, s0, t0, r, g, b, a, x0, y1, 0, s0, t1, r, g, b, a, x1, y1, 0, s1, t1, r, g, b, a, x0, y0, 0, s0, t0, r, g, b, a, x1, y1, 0, s1, t1, r, g, b, a, x1, y0, 0,
					s1, t0, r, g, b, a };

			vector_push_back_data(vVector, vertices, 9 * 6);

			pen->x += glyph->advance_x;
		}
	}
}

MENU_T *menu_new(const char *name, MENU_CALLBACK callback, void *user_data) {
	MENU_T *menu = (MENU_T*) malloc(sizeof(MENU_T));
	memset(menu, 0, sizeof(MENU_T));
	strncpy(menu->name, name, sizeof(menu->name));
	menu->callback = callback;
	menu->user_data = user_data;
	return menu;
}

void menu_delete(MENU_T **_menu) {
	MENU_T *menu = *_menu;
	for (int idx = 0; menu->submenu[idx]; idx++) {
		menu_delete(&menu->submenu[idx]);
	}
	if (menu->callback) {
		menu->callback(menu, MENU_EVENT_BEFORE_DELETE);
	}
	free(menu);
	*_menu = NULL;
}
void expand_menu(MENU_T *menu, vector_t * vVector, int *line_inout, uint32_t screen_width, uint32_t screen_height, int depth) {
	vec2 pen = { };
	vec4 defualt_color = { 1, 1, 1, 1 };
	vec4 activated_color = { 0, 1, 1, 1 };
	vec4 selected_color = { 1, 0, 1, 1 };
	vec4 marked_color = { 1, 1, 0, 1 };
	vec4 back_color = { 0.2, 0.2, 0.2, 1 };
	{
		wchar_t name_w[MENU_NAME_MAX_LENGTH];
		swprintf(name_w, MENU_NAME_MAX_LENGTH, L"%s", menu->name);

		vec4 *color = menu->selected ? &selected_color : menu->activated ? &activated_color : menu->marked ? &marked_color : &defualt_color;
		pen.x = -((float) screen_width / 2 - lg_freetypegles.font->size / 8) + depth * lg_freetypegles.font->size;
		pen.y = ((float) screen_height / 2 - lg_freetypegles.font->size / 8) - lg_freetypegles.font->size * ((*line_inout) + 1);
		add_text(vVector, lg_freetypegles.font, name_w, &back_color, &pen);

		pen.x = -((float) screen_width / 2) + depth * lg_freetypegles.font->size;
		pen.y = ((float) screen_height / 2) - lg_freetypegles.font->size * ((*line_inout) + 1);
		add_text(vVector, lg_freetypegles.font, name_w, color, &pen);
	}
	(*line_inout)++;
	depth++;
	if (menu->selected) {
		for (int idx = 0; menu->submenu[idx]; idx++) {
			expand_menu(menu->submenu[idx], vVector, line_inout, screen_width, screen_height, depth);
		}
	}
}

MENU_T *menu_add_submenu(MENU_T *parent, MENU_T *child, int idx) {
	if (parent == NULL || child == NULL) {
		return NULL;
	}
	int last_idx = 0;
	for (last_idx = 0; parent->submenu[last_idx]; last_idx++) {
		//go to last
	}
	idx = MAX(MIN(idx, last_idx), 0);
	for (int cur = last_idx; cur != idx; cur--) {
		parent->submenu[cur] = parent->submenu[cur - 1];
	}
	parent->submenu[idx] = child;
	child->parent = parent;
	return child;
}
MENU_T *menu_get_submenu(MENU_T *parent, const char *name, bool create_new) {
	for (int idx = 0; parent->submenu[idx]; idx++) {
		if (strncmp(parent->submenu[idx]->name, name, 256) == 0) {
			return parent->submenu[idx];
		}
	}
	if (create_new) {
		return menu_add_submenu(parent, menu_new(name, NULL, NULL), INT_MAX);
	} else {
		return NULL;
	}
}
void menu_operate(MENU_T *root, enum MENU_OPERATE operate) {
	if (root == NULL) {
		return;
	}
	MENU_T *activated_menu = NULL;
	MENU_T *selected_menu = NULL;
	if (!root->activated) {
		if (operate == MENU_OPERATE_ACTIVE_BACK || operate == MENU_OPERATE_ACTIVE_NEXT) {
			root->activated = true;
			if (root->callback) {
				root->callback(root, MENU_EVENT_ACTIVATED);
			}
		}
		return;
	} else {
		activated_menu = root;
	}
	if (root->selected) {
		selected_menu = root;
	}
	if (selected_menu) {
		for (int idx = 0; selected_menu->submenu[idx]; idx++) {
			if (selected_menu->submenu[idx]->activated) {
				activated_menu = selected_menu->submenu[idx];
			}
			if (selected_menu->submenu[idx]->selected) {
				selected_menu = selected_menu->submenu[idx];
				idx = -1;
				continue;
			}
		}
	}
	switch (operate) {
	case MENU_OPERATE_ACTIVE_BACK:
		if (selected_menu && selected_menu != activated_menu) {
			for (int idx = 0; selected_menu->submenu[idx]; idx++) {
				if (selected_menu->submenu[idx] == activated_menu) {
					selected_menu->submenu[idx]->activated = false;
					if (selected_menu->submenu[idx]->callback) {
						selected_menu->submenu[idx]->callback(selected_menu->submenu[idx], MENU_EVENT_DEACTIVATED);
					}
					if (idx == 0) {
						for (; selected_menu->submenu[idx + 1]; idx++) {
							//go to last
						}
					} else {
						idx--;
					}
					selected_menu->submenu[idx]->activated = true;
					if (selected_menu->submenu[idx]->callback) {
						selected_menu->submenu[idx]->callback(selected_menu->submenu[idx], MENU_EVENT_ACTIVATED);
					}
					break;
				}
			}
		}
		break;
	case MENU_OPERATE_ACTIVE_NEXT:
		if (selected_menu && selected_menu != activated_menu) {
			for (int idx = 0; selected_menu->submenu[idx]; idx++) {
				if (selected_menu->submenu[idx] == activated_menu) {
					selected_menu->submenu[idx]->activated = false;
					if (selected_menu->submenu[idx]->callback) {
						selected_menu->submenu[idx]->callback(selected_menu->submenu[idx], MENU_EVENT_DEACTIVATED);
					}
					if (selected_menu->submenu[idx + 1] == NULL) {
						idx = 0;
					} else {
						idx++;
					}
					selected_menu->submenu[idx]->activated = true;
					if (selected_menu->submenu[idx]->callback) {
						selected_menu->submenu[idx]->callback(selected_menu->submenu[idx], MENU_EVENT_ACTIVATED);
					}
					break;
				}
			}
		}
		break;
	case MENU_OPERATE_SELECT:
		if (activated_menu && activated_menu != selected_menu) {
			activated_menu->selected = true;
			if (activated_menu->callback) {
				activated_menu->callback(activated_menu, MENU_EVENT_SELECTED);
			}
			if (activated_menu->submenu[0]) {
				activated_menu->submenu[0]->activated = true;
				if (activated_menu->submenu[0]->callback) {
					activated_menu->submenu[0]->callback(activated_menu->submenu[0], MENU_EVENT_ACTIVATED);
				}
			}
		}
		break;
	case MENU_OPERATE_DESELECT:
		if (selected_menu) {
			selected_menu->selected = false;
			if (selected_menu->callback) {
				selected_menu->callback(selected_menu, MENU_EVENT_DESELECTED);
			}
			for (int idx = 0; selected_menu->submenu[idx]; idx++) {
				if (selected_menu->submenu[idx]->activated) {
					selected_menu->submenu[idx]->activated = false;
					if (selected_menu->submenu[idx]->callback) {
						selected_menu->submenu[idx]->callback(selected_menu->submenu[idx], MENU_EVENT_DEACTIVATED);
					}
				}
			}
		} else if (activated_menu) {
			activated_menu->activated = false;
			if (activated_menu->callback) {
				activated_menu->callback(activated_menu, MENU_EVENT_DEACTIVATED);
			}
		}
		break;
	case MENU_OPERATE_NONE:
	default:
		break;
	}
}
