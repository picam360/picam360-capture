#ifndef PICAM360_TOOLS_H
#define PICAM360_TOOLS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*RECORD_CALLBACK)(unsigned char *data, unsigned int data_len,
		void *user_data);

void *StartRecord(const int width, const int height, const char *filename,
		int bitrate_kbps, int fps, RECORD_CALLBACK callback, void *user_data);
int StopRecord(void *);
int AddFrame(void *, const unsigned char *in_data);
int SaveJpeg(const unsigned char *in_data, const int width, const int height,
		const char *out_filename, int quality);

#ifdef __cplusplus
}
#endif

#endif
