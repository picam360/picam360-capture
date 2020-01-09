#pragma once

OMXJPEG_FN_DEFINE(void, jpeg_CreateDecompress,
		(j_decompress_ptr cinfo, int version, size_t structsize));
OMXJPEG_FN_DEFINE(void, jpeg_destroy_decompress, (j_decompress_ptr cinfo));
OMXJPEG_FN_DEFINE(void, jpeg_mem_src,
		(j_decompress_ptr cinfo, unsigned char * inbuffer, unsigned long insize));
OMXJPEG_FN_DEFINE(int, jpeg_read_header,
		(j_decompress_ptr cinfo, boolean require_image));
OMXJPEG_FN_DEFINE(boolean, jpeg_start_decompress, (j_decompress_ptr cinfo));
OMXJPEG_FN_DEFINE(JDIMENSION, jpeg_read_raw_data,
		(j_decompress_ptr cinfo, JSAMPIMAGE data, JDIMENSION max_lines));
OMXJPEG_FN_DEFINE(boolean, jpeg_finish_decompress, (j_decompress_ptr cinfo));
