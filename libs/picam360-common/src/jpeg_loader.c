#include <stdlib.h>
#include <stdio.h>

#include <jpeglib.h>
#include <jerror.h>

#include "jpeg_loader.h"

bool load_jpeg(const char *file_name, uint8_t **pixels_p, uint32_t *width_p, uint32_t *height_p,
		uint32_t *stride_p) {
	uint8_t *pixels;
	uint32_t width;
	uint32_t height;
	uint32_t stride;

	FILE *file = fopen(file_name, "rb"); //open the file
	struct jpeg_decompress_struct info; //the jpeg decompress info
	struct jpeg_error_mgr err; //the error handler

	info.err = jpeg_std_error(&err); //tell the jpeg decompression handler to send the errors to err
	jpeg_create_decompress(&info); //sets info to all the default stuff

	//if the jpeg file didnt load exit
	if (!file) {
		fprintf(stderr, "Error reading JPEG file %s!!!", file_name);
		return false;
	}

	jpeg_stdio_src(&info, file); //tell the jpeg lib the file we’er reading

	jpeg_read_header(&info, TRUE); //tell it to start reading it

	jpeg_start_decompress(&info); //decompress the file

	width = info.output_width;
	height = info.output_height;
	stride = width * 3;

	//read turn the uncompressed data into something ogl can read
	pixels = (uint8_t*)malloc(height*stride); //setup data for the data its going to be handling

	uint8_t *p1 = pixels;
	uint8_t **p2 = &p1;
	int numlines = 0;

	while (info.output_scanline < info.output_height) {
		numlines = jpeg_read_scanlines(&info, p2, 1);
		*p2 += numlines * 3 * info.output_width;
	}

	jpeg_finish_decompress(&info); //finish decompressing this file

	fclose(file); //close the file

	*pixels_p = pixels;
	*width_p = width;
	*height_p = height;
	*stride_p = stride;

	return true;
}
