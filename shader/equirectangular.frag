varying vec2 tcoord;
uniform mat4 unif_matrix;
uniform sampler2D cam_texture;
uniform sampler2D logo_texture;
uniform float pixel_size;
uniform float split;
//options start
uniform float sharpness_gain;
uniform float cam_offset_yaw;
uniform float cam_offset_x;
uniform float cam_offset_y;
uniform float cam_horizon_r;
//options end

const float M_PI = 3.1415926535;
const float color_offset = 0.10;
const float color_factor = 1.0 / (1.0 - color_offset);

void main(void) {
	float u = 0.0;
	float v = 0.0;
	vec4 pos = vec4(0.0, 0.0, 0.0, 1.0);
	float pitch_orig = -M_PI / 2.0 + M_PI * tcoord.y;
	float yaw_orig;
	if (split == 0.0) {
		yaw_orig = 2.0 * M_PI * tcoord.x - M_PI;
	} else {
		yaw_orig = 2.0 * M_PI * (tcoord.x / 2.0 + 0.5 * (split - 1.0)) - M_PI;
	}
	pos.x = cos(pitch_orig) * sin(yaw_orig); //yaw starts from z
	pos.y = sin(pitch_orig);
	pos.z = cos(pitch_orig) * cos(yaw_orig); //yaw starts from z
	pos = unif_matrix * pos;
	float pitch = asin(pos.y);
	float yaw = atan(pos.x, pos.z); //yaw starts from z

	float r = (M_PI / 2.0 - pitch) / M_PI;
	if (r > 0.65) {
		float yaw2 = -yaw;
		r = (1.0 - r) / 0.35 * 0.5;
		u = r * cos(yaw2) + 0.5;
		v = r * sin(yaw2) + 0.5;
		gl_FragColor = texture2D(logo_texture, vec2(u, v));
		return;
	} else if (r >= 0.55) {
		r = pow(r - 0.55, 1.2) + pow(0.05, 1.1) + pow(0.10, 1.09) + 0.4;
	} else if (r >= 0.50) {
		r = pow(r - 0.50, 1.1) + pow(0.10, 1.09) + 0.4;
	} else if (r >= 0.40) {
		r = pow(r - 0.4, 1.09) + 0.4;
	}

	float yaw2 = yaw + M_PI + cam_offset_yaw;
	u = cam_horizon_r * r * cos(yaw2) + 0.5 + cam_offset_x;
	v = cam_horizon_r * r * sin(yaw2) + 0.5 + cam_offset_y;
	if (u <= 0.0 || u > 1.0 || v <= 0.0 || v > 1.0) {
		u = 0.0;
		v = 0.0;
	}
	if (u == 0.0 && v == 0.0) {
		gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
	} else {
		vec4 fc;
		if (sharpness_gain == 0.0) {
			fc = texture2D(cam_texture, vec2(u, v));
		} else {
			//sharpness
			float gain = sharpness_gain + r/2.0;
			fc = texture2D(cam_texture, vec2(u, v))
					* (1.0 + 4.0 * gain);
			fc -= texture2D(cam_texture, vec2(u - 1.0 * pixel_size, v))
					* gain;
			fc -= texture2D(cam_texture, vec2(u, v - 1.0 * pixel_size))
					* gain;
			fc -= texture2D(cam_texture, vec2(u, v + 1.0 * pixel_size))
					* gain;
			fc -= texture2D(cam_texture, vec2(u + 1.0 * pixel_size, v))
					* gain;
		}

		fc = (fc - color_offset) * color_factor;
		if (r >= 0.45) {
			float r_r = pow(r - 0.45, 1.006) + 0.45;
			u = cam_horizon_r * r_r * cos(yaw2) + 0.5 + cam_offset_x;
			v = cam_horizon_r * r_r * sin(yaw2) + 0.5 + cam_offset_y;
			vec4 fc_b = texture2D(cam_texture, vec2(u, v));

			fc_b = (fc_b - color_offset) * color_factor;
			fc.z = fc_b.z;

			r_r = pow(r - 0.45, 1.003) + 0.45;
			u = cam_horizon_r * r_r * cos(yaw2) + 0.5 + cam_offset_x;
			v = cam_horizon_r * r_r * sin(yaw2) + 0.5 + cam_offset_y;
			fc_b = texture2D(cam_texture, vec2(u, v));

			fc_b = (fc_b - color_offset) * color_factor;
			fc.y = fc_b.y;

		}

		gl_FragColor = fc;
	}
}

