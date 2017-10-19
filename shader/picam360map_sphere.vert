attribute vec4 vPosition; //[0:1]
uniform float scale_x;
uniform float scale_y;
uniform float frame_aspect_ratio;

uniform mat4 unif_matrix;
uniform mat4 unif_matrix_1;
uniform sampler2D cam0_texture;
uniform sampler2D cam1_texture;
uniform float pixel_size;
uniform float cam_aspect_ratio;
//angular map params
uniform float r_2_pitch[256];
//options start
uniform float sharpness_gain;
uniform float cam0_offset_yaw;
uniform float cam0_offset_x;
uniform float cam0_offset_y;
uniform float cam0_horizon_r;
uniform float cam0_aov;
uniform float cam1_offset_yaw;
uniform float cam1_offset_x;
uniform float cam1_offset_y;
uniform float cam1_horizon_r;
uniform float cam1_aov;
//options end

const float M_PI = 3.1415926535;
const float M_PI_DIV_2 = M_PI / 2.0;
const float M_PI_DIV_4 = M_PI / 4.0;
const float M_SQRT_2 = 1.4142135623;

varying float r0;
varying float r1;
varying float u0;
varying float v0;
varying float u1;
varying float v1;

void main(void) {
	vec4 position;
	position.xy = vPosition.xy * vec2(2.0, 2.0) + vec2(-1.0, -1.0); //[-1:1]
	gl_Position.xy = vec2(position.x, position.y * frame_aspect_ratio);
	gl_Position.zw = vec2(1.0, 1.0);

	float r = sqrt(position.x * position.x + position.y * position.y);
	if (r > M_SQRT_2) {
		r = M_SQRT_2;
	}
	float indexf = r / M_SQRT_2 * 255.0;
	int index = int(indexf);
	float index_sub = indexf - float(index);
	float pitch_orig = r_2_pitch[index] * (1.0 - index_sub) + r_2_pitch[index + 1] * index_sub;
	float roll_orig = atan(position.y, position.x);
	if (r < M_SQRT_2 && r > 1.0) {
		int roll_index = int(roll_orig / M_PI_DIV_2);
		float roll_base = float(roll_index) * M_PI_DIV_2 + (roll_orig > 0.0 ? M_PI_DIV_4 : -M_PI_DIV_4);
		float roll_diff = roll_orig - roll_base;
		float roll_gain = M_PI / (M_PI - 4.0 * acos(1.0 / r));
		roll_orig = roll_diff * roll_gain + roll_base;
	}
	position.x = sin(pitch_orig) * cos(roll_orig);
	position.y = sin(pitch_orig) * sin(roll_orig);
	position.z = cos(pitch_orig);
	position.w = 1.0;

	{
		vec4 pos = unif_matrix * position;
		float pitch = acos(pos.y);
		float yaw = atan(pos.x, pos.z); //yaw starts from z

		r0 = pitch / M_PI;
		float r2 = r0;
		r2 = sin(M_PI * 180.0 / cam0_aov * r2) / 2.0;
		float yaw2 = yaw + M_PI + cam0_offset_yaw;
		u0 = cam0_horizon_r * r2 / cam_aspect_ratio * cos(yaw2) + 0.5 + cam0_offset_x;
		v0 = cam0_horizon_r * r2 * sin(yaw2) + 0.5 + cam0_offset_y;
	}
	{
		vec4 pos = unif_matrix_1 * position;
		float pitch = acos(pos.y);
		float yaw = atan(pos.x, pos.z);

		r1 = pitch / M_PI;
		float r2 = 1.0 - r1;
		r2 = sin(M_PI * 180.0 / cam1_aov * r2) / 2.0;
		float yaw2 = -yaw + M_PI + cam1_offset_yaw;
		u1 = cam1_horizon_r * r2 / cam_aspect_ratio * cos(yaw2) + 0.5 + cam1_offset_x;
		v1 = cam1_horizon_r * r2 * sin(yaw2) + 0.5 + cam1_offset_y;
	}
}
