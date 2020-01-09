#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "jpeglib.h"

#define OMXJPEG_FN_DEFINE(ret_type, fnc_name, args) ret_type omxjpeg_##fnc_name args


#include "omx_jpeg_compress.h"
#include "omx_jpeg_decompress.h"

OMXJPEG_FN_DEFINE(struct jpeg_error_mgr*, jpeg_std_error,
		(struct jpeg_error_mgr * err));

#ifdef __cplusplus
}
#endif
