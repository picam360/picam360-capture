#include "omxjpeg.h"

OMXJPEG_FN_DEFINE(struct jpeg_error_mgr*, jpeg_std_error,
		(struct jpeg_error_mgr * err)){
	return NULL;
}
