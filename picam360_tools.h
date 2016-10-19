#ifndef PICAM360_TOOLS_H
#define PICAM360_TOOLS_H


#ifdef __cplusplus
extern "C" {
#endif

int TransformToEquirectangular(int texture_width, int texture_height,
		int equirectangular_width, int equirectangular_height,
		const unsigned char *in_data, unsigned char *out_data);
int StartRecord(const char *filename, int bitrate_kbps);
int StopRecord();
int AddFrame(const unsigned char *in_data);
int SaveJpeg(const unsigned char *in_data, const char *out_filename, int quality);
int SetRotation(float x_deg, float y_deg, float z_deg);

#ifdef __cplusplus
}
#endif

#endif
