#ifndef PICAM360_TOOLS_H
#define PICAM360_TOOLS_H


#ifdef __cplusplus
extern "C" {
#endif

int StartRecord(const int width, const int height, const char *filename, int bitrate_kbps);
int StopRecord();
int AddFrame(const unsigned char *in_data);
int SaveJpeg(const unsigned char *in_data, const int width, const int height, const char *out_filename, int quality);

#ifdef __cplusplus
}
#endif

#endif
