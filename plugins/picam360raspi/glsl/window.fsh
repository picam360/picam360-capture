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
uniform cam_sampler2D cam_texture[MAX_NUM_OF_CAM];
uniform float cam_aov[MAX_NUM_OF_CAM];
uniform sampler2D logo_texture;
uniform float color_offset;
uniform float color_factor;
uniform float overlap;

IN vec3 cam_uvr[MAX_NUM_OF_CAM];
IN vec3 logo_uvr;

void main(void) {
	vec4 fcs[MAX_NUM_OF_CAM];
	float alpha = 0.0;
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
}