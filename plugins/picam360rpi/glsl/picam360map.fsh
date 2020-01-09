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

#define NUM_OF_CAM %NUM_OF_CAM%

precision highp float;

const float M_PI = 3.1415926535;

uniform cam_sampler2D cam_texture[NUM_OF_CAM];
uniform float cam_aov[NUM_OF_CAM];
uniform sampler2D logo_texture;
uniform float color_offset;
uniform float color_factor;
uniform float overlap;

IN vec3 logo_uvr;
IN vec3 cam_uvr[NUM_OF_CAM];

#define FCS(i) \
	{\
		float r_thresh = cam_aov[i] / 360.0;\
		if (cam_uvr[i][2] > r_thresh || cam_uvr[i][0] < 0.0 || cam_uvr[i][0] > 1.0 || cam_uvr[i][1] < 0.0 || cam_uvr[i][1] > 1.0) {\
			fcs[i] = vec4(0.0, 0.0, 0.0, 0.0);\
		} else {\
			fcs[i] = texture2D(cam_texture[i], vec2(cam_uvr[i][0], cam_uvr[i][1]));\
			fcs[i].a = 1.0 - cam_uvr[i][2] / r_thresh;\
			alpha += fcs[i].a;\
		}\
	}

void main(void) {
	vec4 fcs[NUM_OF_CAM];
	float alpha = 0.0;
	%FOR_FCS_NUM_OF_CAM%
	if (alpha == 0.0) {
		gl_FragColor = texture2D(logo_texture, vec2(logo_uvr.x, logo_uvr.y));
	} else {
		vec4 fc = vec4(0.0, 0.0, 0.0, 0.0);
		{
			const int i = 0;
			fc += fcs[i] * (fcs[i].a / alpha);
		}
#if (NUM_OF_CAM > 1)
		{
			const int i = 1;
			fc += fcs[i] * (fcs[i].a / alpha);
		}
#endif
		fc = (fc - color_offset) * color_factor;
		gl_FragColor = fc;
	}
}
