#if (__VERSION__ > 120)
#define IN in
#define OUT out
#define texture2D texture
#define gl_FragColor FragColor
layout (location=0) out vec4 FragColor;
#else
#define IN varying
#define OUT varying
#endif // __VERSION
precision highp float;

const int MAX_NUM_OF_CAM = 3;
const float M_PI = 3.1415926535;

uniform int num_of_cam;
uniform sampler2D cam_texture[MAX_NUM_OF_CAM];
uniform float cam_aov[MAX_NUM_OF_CAM];
uniform sampler2D logo_texture;
uniform float color_offset;
uniform float color_factor;
uniform float overlap;

IN vec3 cam_uvr[MAX_NUM_OF_CAM];
IN vec3 logo_uvr;

#ifdef TEGRA
const float tex_width = 4192.0;
const float tex_height = 3120.0;
uniform mat4 YUV2RGB;

vec4 get_yuv(sampler2D texture, float u, float v, float pitch_u, float pitch_v, int smooth) {
	vec4 yuv;
	vec4 pixel_0_0 = texture2D(texture, vec2(u, v));
	if (smooth == 0) {
		int cur = int(mod(u / pitch_u, 2.0));

		yuv[0] = pixel_0_0[cur * 2 + 1];
		yuv[1] = pixel_0_0[0];
		yuv[2] = pixel_0_0[2];
		yuv[3] = 1.0;
	} else {
		vec4 pixel_0_1 = texture2D(texture, vec2(u, v + pitch_v));
		float cur_y = mod(v / pitch_v, 1.0);
		float cur_x = mod(u / pitch_u, 2.0);
		if (cur_x > 1.0) {
			vec4 pixel_1_0 = texture2D(texture, vec2(u + pitch_u, v));
			vec4 pixel_1_1 = texture2D(texture, vec2(u + pitch_u, v + pitch_v));
			vec4 tmp;
			float wl = 2.0 - cur_x;
			float wr = cur_x - 1.0;
			float wt = 1.0 - cur_y;
			float wb = cur_y;

			yuv[0] = pixel_0_0[3] * wl;
			yuv[1] = pixel_0_0[0] * wl;
			yuv[2] = pixel_0_0[2] * wl;
			yuv[0] += pixel_1_0[1] * wr;
			yuv[1] += pixel_1_0[0] * wr;
			yuv[2] += pixel_1_0[2] * wr;

			tmp[0] = pixel_0_1[3] * wl;
			tmp[1] = pixel_0_1[0] * wl;
			tmp[2] = pixel_0_1[2] * wl;
			tmp[0] += pixel_1_1[1] * wr;
			tmp[1] += pixel_1_1[0] * wr;
			tmp[2] += pixel_1_1[2] * wr;
			for (int i = 0; i < 3; i++) {
				yuv[i] = yuv[i] * wt + tmp[i] * wb;
			}
			yuv[3] = 1.0;
		} else {
			float tmp[3];
			float wl = 1.0 - cur_x;
			float wr = cur_x;
			float wt = 1.0 - cur_y;
			float wb = cur_y;

			yuv[0] = pixel_0_0[1] * wl;
			yuv[1] = pixel_0_0[0] * wl;
			yuv[2] = pixel_0_0[2] * wl;
			yuv[0] += pixel_0_0[3] * wr;
			yuv[1] += pixel_0_0[0] * wr;
			yuv[2] += pixel_0_0[2] * wr;

			tmp[0] = pixel_0_1[1] * wl;
			tmp[1] = pixel_0_1[0] * wl;
			tmp[2] = pixel_0_1[2] * wl;
			tmp[0] += pixel_0_1[3] * wr;
			tmp[1] += pixel_0_1[0] * wr;
			tmp[2] += pixel_0_1[2] * wr;
			for (int i = 0; i < 3; i++) {
				yuv[i] = yuv[i] * wt + tmp[i] * wb;
			}
			yuv[3] = 1.0;
		}
	}
	return yuv;
}
#endif

void main(void) {
	vec4 fcs[MAX_NUM_OF_CAM];
	float alpha = 0.0;
#ifdef TEGRA
	for (int i = 0; i < num_of_cam; i++) {
		float r_thresh = cam_aov[i] / 360.0;
		if (cam_uvr[i][2] < r_thresh && cam_uvr[i][0] >= 0.0 && cam_uvr[i][0] <= 1.0 && cam_uvr[i][1] >= 0.0 && cam_uvr[i][1] <= 1.0) {
			vec4 yuv = get_yuv(cam_texture[i], cam_uvr[i][0], cam_uvr[i][1], 1.0 / tex_width, 1.0 / tex_height, 1);
			fcs[i] = yuv * YUV2RGB;
			fcs[i].a = 1.0 - cam_uvr[i][2] / r_thresh;
			alpha += fcs[i].a;
		} else {
			fcs[i] = vec4(0.0, 0.0, 0.0, 0.0);
		}
	}
	if (alpha == 0.0) {
		gl_FragColor = texture2D(logo_texture, vec2(logo_uvr.x, logo_uvr.y));
	} else {
		vec4 fc = vec4(0.0, 0.0, 0.0, 0.0);
		for (int i = 0; i < num_of_cam; i++) {
			fc += fcs[i] * (fcs[i].a / alpha);
		}
		fc = (fc - color_offset) * color_factor;
		gl_FragColor = fc;
	}
#else
	//if(num_of_cam >= 1)
	{
		const int i = 0;
		float r_thresh = cam_aov[i] / 360.0;
		if (cam_uvr[i][2] > r_thresh || cam_uvr[i][0] <= 0.0 || cam_uvr[i][0] > 1.0 || cam_uvr[i][1] <= 0.0 || cam_uvr[i][1] > 1.0) {
			fcs[i] = vec4(0.0, 0.0, 0.0, 0.0);
		} else {
			fcs[i] = texture2D(cam_texture[i], vec2(cam_uvr[i][0], cam_uvr[i][1]));
			fcs[i].a = 1.0 - cam_uvr[i][2] / r_thresh;
			alpha += fcs[i].a;
		}
	}
	if (num_of_cam >= 2) {
		const int i = 1;
		float r_thresh = cam_aov[i] / 360.0;
		if (cam_uvr[i][2] > r_thresh || cam_uvr[i][0] <= 0.0 || cam_uvr[i][0] > 1.0 || cam_uvr[i][1] <= 0.0 || cam_uvr[i][1] > 1.0) {
			fcs[i] = vec4(0.0, 0.0, 0.0, 0.0);
		} else {
			fcs[i] = texture2D(cam_texture[i], vec2(cam_uvr[i][0], cam_uvr[i][1]));
			fcs[i].a = 1.0 - cam_uvr[i][2] / r_thresh;
			alpha += fcs[i].a;
		}
	}
	if (alpha == 0.0) {
		gl_FragColor = texture2D(logo_texture, vec2(logo_uvr.x, logo_uvr.y));
	} else {
		vec4 fc = vec4(0.0, 0.0, 0.0, 0.0);
		//if(num_of_cam >= 1)
		{
			const int i = 0;
			fc += fcs[i] * (fcs[i].a / alpha);
		}
		if (num_of_cam >= 2) {
			const int i = 1;
			fc += fcs[i] * (fcs[i].a / alpha);
		}
		fc = (fc - color_offset) * color_factor;
		gl_FragColor = fc;
	}
#endif
}
