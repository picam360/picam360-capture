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

precision mediump float;

const int MAX_NUM_OF_CAM = 3;
IN vec2 tcoord;
uniform sampler2D logo_texture;
uniform sampler2D cam_texture[MAX_NUM_OF_CAM];
uniform float cam_offset_x[MAX_NUM_OF_CAM];
uniform float cam_offset_y[MAX_NUM_OF_CAM];
uniform float cam_horizon_r[MAX_NUM_OF_CAM];
uniform float pixel_size;
uniform int active_cam;
uniform float sharpness_gain;
uniform float cam_aspect_ratio;

void main(void) {
	vec4 fc = texture2D(logo_texture, vec2(tcoord.x, tcoord.y));
	if (fc.g == 0.0) {
#ifdef TEGRA
		float u = (tcoord.x + cam_offset_x[active_cam] - 0.5)/cam_aspect_ratio + 0.5;
		float v = tcoord.y + cam_offset_y[active_cam];
		if (sharpness_gain == 0.0) {
			fc = texture2D(cam_texture[active_cam], vec2(u, v));
		} else {
			//sharpness
			float gain = sharpness_gain;
			fc = texture2D(cam_texture[active_cam], vec2(u, v))
					* (1.0 + 4.0 * gain);
			fc -= texture2D(cam_texture[active_cam], vec2(u - 1.0 * pixel_size, v))
					* gain;
			fc -= texture2D(cam_texture[active_cam], vec2(u, v - 1.0 * pixel_size))
					* gain;
			fc -= texture2D(cam_texture[active_cam], vec2(u, v + 1.0 * pixel_size))
					* gain;
			fc -= texture2D(cam_texture[active_cam], vec2(u + 1.0 * pixel_size, v))
					* gain;
		}
#else
		if(active_cam == 0) {
			const int i = 0;
			float u = (tcoord.x + cam_offset_x[i] - 0.5)/cam_aspect_ratio + 0.5;
			float v = tcoord.y + cam_offset_y[i];
			if (sharpness_gain == 0.0) {
				fc = texture2D(cam_texture[i], vec2(u, v));
			} else {
				//sharpness
				float gain = sharpness_gain;
				fc = texture2D(cam_texture[i], vec2(u, v))
				* (1.0 + 4.0 * gain);
				fc -= texture2D(cam_texture[i], vec2(u - 1.0 * pixel_size, v))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u, v - 1.0 * pixel_size))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u, v + 1.0 * pixel_size))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u + 1.0 * pixel_size, v))
				* gain;
			}
		} else if(active_cam == 1) {
			const int i = 1;
			float u = (tcoord.x + cam_offset_x[i] - 0.5)/cam_aspect_ratio + 0.5;
			float v = tcoord.y + cam_offset_y[i];
			if (sharpness_gain == 0.0) {
				fc = texture2D(cam_texture[i], vec2(u, v));
			} else {
				//sharpness
				float gain = sharpness_gain;
				fc = texture2D(cam_texture[i], vec2(u, v))
				* (1.0 + 4.0 * gain);
				fc -= texture2D(cam_texture[i], vec2(u - 1.0 * pixel_size, v))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u, v - 1.0 * pixel_size))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u, v + 1.0 * pixel_size))
				* gain;
				fc -= texture2D(cam_texture[i], vec2(u + 1.0 * pixel_size, v))
				* gain;
			}
		}
#endif
	}
	gl_FragColor = fc;
}