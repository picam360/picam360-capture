#pragma once

OMXJPEG_FN_DEFINE(void, jpeg_CreateCompress,
		(j_compress_ptr cinfo, int version, size_t structsize));
OMXJPEG_FN_DEFINE(void, jpeg_destroy_compress, (j_compress_ptr cinfo));
OMXJPEG_FN_DEFINE(void, jpeg_suppress_tables,
		(j_compress_ptr cinfo, boolean suppress));
OMXJPEG_FN_DEFINE(void, jpeg_mem_dest,
		(j_compress_ptr cinfo, unsigned char ** outbuffer,unsigned long * outsize));
OMXJPEG_FN_DEFINE(void, jpeg_set_defaults, (j_compress_ptr cinfo));
OMXJPEG_FN_DEFINE(void, jpeg_set_quality,
		(j_compress_ptr cinfo, int quality, boolean force_baseline));
OMXJPEG_FN_DEFINE(void, jpeg_set_hardware_acceleration_parameters_enc,
		(j_compress_ptr cinfo, boolean hw_acceleration, unsigned int defaultBuffSize, unsigned int defaultWidth, unsigned int defaultHeight ));
OMXJPEG_FN_DEFINE(JDIMENSION, jpeg_write_raw_data,
		(j_compress_ptr cinfo, JSAMPIMAGE data, JDIMENSION num_lines));
OMXJPEG_FN_DEFINE(void, jpeg_start_compress,
		(j_compress_ptr cinfo, boolean write_all_tables));
OMXJPEG_FN_DEFINE(void, jpeg_finish_compress, (j_compress_ptr cinfo));
