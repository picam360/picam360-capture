varying vec2 tcoord;
uniform mat4 unif_matrix;
uniform sampler2D cam0_texture;
uniform sampler2D cam1_texture;
uniform float pixel_size;
//options start
uniform float sharpness_gain;
uniform float cam0_offset_yaw;
uniform float cam0_offset_x;
uniform float cam0_offset_y;
uniform float cam0_horizon_r;
uniform float cam1_offset_yaw;
uniform float cam1_offset_x;
uniform float cam1_offset_y;
uniform float cam1_horizon_r;
//options end

const float M_PI = 3.1415926535;
//const float aspect = 480.0 / 640.0;
const float aspect = 1.0;
const float color_offset = 0.15;
const float color_factor = 1.0 / (1.0 - color_offset);

void main(void) {
	float u = 0.0;
	float v = 0.0;
	vec4 pos = vec4(0.0, 0.0, 0.0, 1.0);
	float roll_orig = M_PI / 2.0 - M_PI * tcoord.y;
	float yaw_orig = 2.0 * M_PI * tcoord.x - M_PI;
	pos.x = cos(roll_orig) * sin(yaw_orig); //yaw starts from y
	pos.y = cos(roll_orig) * cos(yaw_orig); //yaw starts from y
	pos.z = sin(roll_orig);
	pos = unif_matrix * pos;
	float roll = -asin(pos.z); //this is for jpeg cordinate
	float yaw = atan(pos.x, pos.y); //yaw starts from y

	float r = (M_PI / 2.0 - roll) / M_PI;
	if (r < 0.5) {
		if (r >= 0.40) {
			r = pow(r - 0.4, 1.09) + 0.4;
		}
		float yaw2 = -yaw + M_PI + cam0_offset_yaw;
		float u_factor = aspect * cam0_horizon_r;
		float v_factor = cam0_horizon_r;
		u = u_factor * r * cos(yaw2) + 0.5 + cam0_offset_x;
		v = v_factor * r * sin(yaw2) + 0.5 - cam0_offset_y; //cordinate is different
		if (u <= 0.0 || u > 1.0 || v <= 0.0 || v > 1.0) {
			u = 0.0;
			v = 0.0;
		}
		if (u == 0.0 && v == 0.0) {
			gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
		} else {
			vec4 fc;
			if (sharpness_gain == 0.0) {
				fc = texture2D(cam0_texture, vec2(u, v));
			} else {
				//sharpness
				fc = texture2D(cam0_texture, vec2(u, v))
						* (1.0 + 4.0 * sharpness_gain);
				fc -= texture2D(cam0_texture, vec2(u - 1.0 * pixel_size, v))
						* sharpness_gain;
				fc -= texture2D(cam0_texture, vec2(u, v - 1.0 * pixel_size))
						* sharpness_gain;
				fc -= texture2D(cam0_texture, vec2(u, v + 1.0 * pixel_size))
						* sharpness_gain;
				fc -= texture2D(cam0_texture, vec2(u + 1.0 * pixel_size, v))
						* sharpness_gain;
			}

			gl_FragColor = fc;
		}
	} else {
		r = 1.0 - r;
		if (r >= 0.40) {
			r = pow(r - 0.4, 1.09) + 0.4;
		}
		float yaw2 = -yaw + M_PI;
		float u_factor = aspect * cam1_horizon_r;
		float v_factor = cam1_horizon_r;
		u = u_factor * r * cos(yaw2) + 0.5 + cam1_offset_x;
		v = v_factor * r * sin(yaw2) + 0.5 - cam1_offset_y; //cordinate is different
		if (u <= 0.0 || u > 1.0 || v <= 0.0 || v > 1.0) {
			u = 0.0;
			v = 0.0;
		}
		if (u == 0.0 && v == 0.0) {
			gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
		} else {
			vec4 fc;
			if (sharpness_gain == 0.0) {
				fc = texture2D(cam1_texture, vec2(u, v));
			} else {
				//sharpness
				fc = texture2D(cam1_texture, vec2(u, v))
						* (1.0 + 4.0 * sharpness_gain);
				fc -= texture2D(cam1_texture, vec2(u - 1.0 * pixel_size, v))
						* sharpness_gain;
				fc -= texture2D(cam1_texture, vec2(u, v - 1.0 * pixel_size))
						* sharpness_gain;
				fc -= texture2D(cam1_texture, vec2(u, v + 1.0 * pixel_size))
						* sharpness_gain;
				fc -= texture2D(cam1_texture, vec2(u + 1.0 * pixel_size, v))
						* sharpness_gain;
			}

			gl_FragColor = fc;
		}
	}

}

