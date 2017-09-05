attribute vec4 vPosition;
uniform float scale_x;
uniform float scale_y;
uniform float frame_aspect_ratio;

uniform mat4 unif_matrix;
uniform mat4 unif_matrix_1;
uniform sampler2D cam0_texture;
uniform sampler2D cam1_texture;
uniform float pixel_size;
uniform float cam_aspect_ratio;
uniform float split;
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

varying float r0;
varying float r1;
varying float u0;
varying float v0;
varying float u1;
varying float v1;

void main(void) {
	vec4 position = vPosition;
	gl_Position.xy = position.xy * vec2(2, 2) + vec2(-1, -1);
	gl_Position.xy = vec2(gl_Position.x * scale_x, gl_Position.y * scale_y * frame_aspect_ratio);
	gl_Position.zw = vec2(1.0, 1.0);

	float pitch_orig = -M_PI / 2.0 + M_PI * position.y;
	float yaw_orig;
	if (split == 0.0) {
		yaw_orig = 2.0 * M_PI * position.x - M_PI;
	} else {
		yaw_orig = 2.0 * M_PI * (position.x / 2.0 + 0.5 * (split - 1.0)) - M_PI;
	}
	position.x = cos(pitch_orig) * sin(yaw_orig); //yaw starts from z
	position.y = sin(pitch_orig);
	position.z = cos(pitch_orig) * cos(yaw_orig); //yaw starts from z
	position.w = 1.0;

	vec4 pos = unif_matrix * position;
	float pitch = asin(pos.y);
	float yaw = atan(pos.x, pos.z); //yaw starts from z

	{
		r0 = (M_PI / 2.0 - pitch) / M_PI;
		float r2 = r0;
		r2 = sin(M_PI * 180.0 / cam0_aov * r2) / 2.0;
		float yaw2 = yaw + M_PI + cam0_offset_yaw;
		u0 = cam0_horizon_r / cam_aspect_ratio * r2 * cos(yaw2) + 0.5 + cam0_offset_x;
		v0 = cam0_horizon_r * r2 * sin(yaw2) + 0.5 + cam0_offset_y;
	}
	{
		float yaw2 = -yaw;
		r1 = (1.0 - r0) / 0.35 * 0.5;
		u1 = r1 * cos(yaw2) + 0.5;
		v1 = r1 * sin(yaw2) + 0.5;
	}
}
