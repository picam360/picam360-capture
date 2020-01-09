#pragma once

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

void init_menu(uint32_t font_size);
void deinit_menu();

void menu_redraw(MENU_T *root, char *_status, uint32_t screen_width,
		uint32_t screen_height, uint32_t frame_width, uint32_t frame_height,
		bool stereo);
