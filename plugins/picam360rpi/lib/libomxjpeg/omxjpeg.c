#include "omxjpeg.h"

OMXJPEG_FN_DEFINE(struct jpeg_error_mgr*, jpeg_std_error,
		(struct jpeg_error_mgr * err)){
	memset(err, 0, sizeof(struct jpeg_error_mgr));
	return err;
}
