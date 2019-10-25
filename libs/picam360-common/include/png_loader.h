#pragma once

#include <stdbool.h>
#include <stdint.h>


bool load_png(const char *file_name, uint8_t **pixels, uint32_t *width, uint32_t *height, uint32_t *stride);
