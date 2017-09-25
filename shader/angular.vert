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
//coner pitch PI : angular_r = M_SQRT_2 && angular_gain = M_PI / asin(1.0 / angular_r) 
//mirror-ball : angular_r = 1.0 && angular_gain = M_PI / asin(1.0 / angular_r) 
//axsis edge fov_rad : angular_r = 1.0 && angular_gain = fov_rad / asin(1.0 / angular_r) 
uniform float angular_gain;
uniform float angular_r;
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
	float pitch_orig = angular_gain * asin(r / angular_r) ;
	float roll_orig = atan(position.y, position.x);
	position.x = sin(pitch_orig) * cos(roll_orig);
	position.y = sin(pitch_orig) * sin(roll_orig);
	position.z = cos(pitch_orig);
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
		u1 = pos.x / -pos.y * 0.35 + 0.5;
		v1 = pos.z / -pos.y * 0.35 + 0.5;
	}
}
