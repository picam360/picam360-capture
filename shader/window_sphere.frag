varying vec4 position;

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

const float overlap = 0.3;
const float M_PI = 3.1415926535;

void main(void) {
	float u = 0.0;
	float v = 0.0;
	vec4 pos = unif_matrix * position;
	float roll = -asin(pos.y); //this is for jpeg coordinate
	float yaw = atan(pos.x, pos.z);

	vec4 fc0;
	vec4 fc1;
	float r = (M_PI / 2.0 - roll) / M_PI;
	if (r < 0.55) {
		float r2 = r;
		if (r2 >= 0.40) {
			r2 = pow(r2 - 0.4, 1.09) + 0.4;
		}
		float yaw2 = -yaw + M_PI + cam0_offset_yaw;
		u = cam0_horizon_r * r2 * cos(yaw2) + 0.5 + cam0_offset_x;
		v = cam0_horizon_r * r2 * sin(yaw2) + 0.5 - cam0_offset_y; //cordinate is different
		if (u <= 0.0 || u > 1.0 || v <= 0.0 || v > 1.0) {
			u = 0.0;
			v = 0.0;
		}
		if (u == 0.0 && v == 0.0) {
			fc0 = vec4(0.0, 0.0, 0.0, 1.0);
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

			fc0 = fc;
		}
	}
	if (r > 0.45) {
		float r2 = 1.0 - r;
		if (r2 >= 0.40) {
			r2 = pow(r2 - 0.4, 1.09) + 0.4;
		}
		float yaw2 = yaw + M_PI + cam1_offset_yaw;
		u = cam1_horizon_r * r2 * cos(yaw2) + 0.5 + cam1_offset_x;
		v = cam1_horizon_r * r2 * sin(yaw2) + 0.5 - cam1_offset_y; //cordinate is different
		if (u <= 0.0 || u > 1.0 || v <= 0.0 || v > 1.0) {
			u = 0.0;
			v = 0.0;
		}
		if (u == 0.0 && v == 0.0) {
			fc1 = vec4(0.0, 0.0, 0.0, 1.0);
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

			fc1 = fc;
		}
	}
	if (r < 0.5 - overlap) {
		gl_FragColor = fc0;
	} else if (r < 0.5 - overlap) {
		gl_FragColor = (fc0 * ((0.5 - overlap) - r) + fc1 * (r - (0.5 - overlap))) / (overlap * 2.0);
	} else {
		gl_FragColor = fc1;
	}
}
