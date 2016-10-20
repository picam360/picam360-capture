varying vec2 tcoord;
uniform mat4 unif_matrix;
uniform sampler2D cam0_texture;

const float M_PI = 3.1415926535;
//const float aspect = 480.0 / 640.0;
const float aspect = 1.0;
const float image_r = 0.85;
const vec2 center1 = vec2(0.50, 0.50);
const vec2 center2 = vec2(0.50, 0.50);
const float color_offset = 0.15;
const float color_factor = 1.0 / (1.0 - color_offset);
float k = 1.0;
float step = 1.0 / 2048.0;

void main(void) {
	float u_factor = aspect * image_r;
	float v_factor = image_r;
	float u = 0.0;
	float v = 0.0;
	vec4 pos = vec4(0.0, 0.0, 0.0, 1.0);
	float roll_orig = M_PI / 2.0 - M_PI * tcoord.y;
	float yaw_orig = 2.0 * M_PI * tcoord.x - M_PI;
	pos.x = cos(roll_orig) * sin(yaw_orig); //yaw starts from y
	pos.y = cos(roll_orig) * cos(yaw_orig); //yaw starts from y
	pos.z = sin(roll_orig);
	pos = unif_matrix * pos;
	float roll = asin(pos.z);
	float yaw = atan(pos.x, pos.y); //yaw starts from y

	float r = (M_PI / 2.0 - roll) / M_PI;
	if (r >= 0.55) {
		r = pow(r - 0.55, 1.2) + pow(0.05, 1.1) + pow(0.10, 1.09) + 0.4;
	} else if (r >= 0.50) {
		r = pow(r - 0.50, 1.1) + pow(0.10, 1.09) + 0.4;
	} else if (r >= 0.40) {
		r = pow(r - 0.4, 1.09) + 0.4;
	}

//		r = pow(r/0.5, 0.8) * 0.5;
	float yaw2 = -yaw + M_PI;
	u = u_factor * r * cos(yaw2) + center1.x;
	v = v_factor * r * sin(yaw2) + center1.y;
	if (u <= 0.0 || u > 1.0 || v <= 0.0 || v > 1.0) {
		u = 0.0;
		v = 0.0;
	}
	if (u == 0.0 && v == 0.0) {
		gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
	} else {
		vec4 fc = texture2D(cam0_texture, vec2(u, v));

		fc = (fc - color_offset) * color_factor;
		if (r >= 0.45) {
			float r_r = pow(r - 0.45, 1.006) + 0.45;
			u = u_factor * r_r * cos(yaw2) + center1.x;
			v = v_factor * r_r * sin(yaw2) + center1.y;
			vec4 fc_b = texture2D(cam0_texture, vec2(u, v));

			fc_b = (fc_b - color_offset) * color_factor;
			fc.z = fc_b.z;

			r_r = pow(r - 0.45, 1.003) + 0.45;
			u = u_factor * r_r * cos(yaw2) + center1.x;
			v = v_factor * r_r * sin(yaw2) + center1.y;
			fc_b = texture2D(cam0_texture, vec2(u, v));

			fc_b = (fc_b - color_offset) * color_factor;
			fc.y = fc_b.y;

		}

		gl_FragColor = fc;
	}
}

