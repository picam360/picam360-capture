#include <stdlib.h>
#include <stdio.h>

#include <png.h>

#include "png_loader.h"

bool load_png(const char *file_name, uint8_t **pixels_p, uint32_t *width_p,
		uint32_t *height_p, uint32_t *stride_p) {
	uint8_t *pixels;
	uint32_t width;
	uint32_t height;
	uint32_t stride;
	png_byte color_type;
	png_byte bit_depth;
	png_bytep *row_pointers = NULL;

	FILE *fp = fopen(file_name, "rb");

	png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL,
			NULL);
	if (!png)
		abort();

	png_infop info = png_create_info_struct(png);
	if (!info)
		abort();

	if (setjmp(png_jmpbuf(png)))
		abort();

	png_init_io(png, fp);

	png_read_info(png, info);

	color_type = png_get_color_type(png, info);
	bit_depth = png_get_bit_depth(png, info);

	// Read any color_type into 8bit depth, RGBA format.
	// See http://www.libpng.org/pub/png/libpng-manual.txt

	if (bit_depth == 16)
		png_set_strip_16(png);

	if (color_type == PNG_COLOR_TYPE_PALETTE)
		png_set_palette_to_rgb(png);

	// PNG_COLOR_TYPE_GRAY_ALPHA is always 8 or 16bit depth.
	if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
		png_set_expand_gray_1_2_4_to_8(png);

	if (png_get_valid(png, info, PNG_INFO_tRNS))
		png_set_tRNS_to_alpha(png);

	// These color_type don't have an alpha channel then fill it with 0xff.
	if (color_type == PNG_COLOR_TYPE_RGB || color_type == PNG_COLOR_TYPE_GRAY
			|| color_type == PNG_COLOR_TYPE_PALETTE)
		png_set_filler(png, 0xFF, PNG_FILLER_AFTER);

	if (color_type == PNG_COLOR_TYPE_GRAY
			|| color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
		png_set_gray_to_rgb(png);

	png_read_update_info(png, info);

	width = png_get_image_width(png, info);
	height = png_get_image_height(png, info);
	stride = png_get_rowbytes(png, info);

	pixels = (uint8_t*) malloc(height * stride);
	row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
	for (int y = 0; y < height; y++) {
		row_pointers[y] = (png_byte*) (pixels + stride * y);
	}

	png_read_image(png, row_pointers);

	free(row_pointers);

	fclose(fp);

	png_destroy_read_struct(&png, &info, NULL);

	*pixels_p = pixels;
	*width_p = width;
	*height_p = height;
	*stride_p = stride;
}
