attribute vec4 vPosition;
uniform float scale;
uniform float frame_aspect_ratio;

uniform mat4 unif_matrix;
uniform mat4 unif_matrix_1;
uniform sampler2D cam0_texture;
uniform sampler2D cam1_texture;
uniform float pixel_size;
uniform float cam_aspect_ratio;
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

const float overlap = 0.03;
const float M_PI = 3.1415926535;

varying float r0;
varying float r1;
varying float u0;
varying float v0;
varying float u1;
varying float v1;

void main(void) {
	vec4 position = vPosition;
	gl_Position = vec4(vPosition.x / vPosition.z * scale,
			vPosition.y / vPosition.z * scale * frame_aspect_ratio, 1.0, 1.0);
	

	{
		vec4 pos = unif_matrix * position;
		float pitch = asin(pos.y);
		float yaw = atan(pos.x, pos.z);
	
		r0 = (M_PI / 2.0 - pitch) / M_PI;
		float r2 = r0;
		if (r2 >= 0.40) {
			r2 = pow(r2 - 0.4, 1.09) + 0.4;
		}
		float yaw2 = yaw + M_PI + cam0_offset_yaw;
		u0 = cam0_horizon_r / cam_aspect_ratio * r2 * cos(yaw2) + 0.5 + cam0_offset_x;
		v0 = cam0_horizon_r * r2 * sin(yaw2) + 0.5 + cam0_offset_y;
	}
	{	
		pos = unif_matrix_1 * position;
		pitch = asin(pos.y);
		yaw = atan(pos.x, pos.z);
		r1 = (M_PI / 2.0 - pitch) / M_PI;
	
		float r2 = 1.0 - r1;
		if (r2 >= 0.40) {
			r2 = pow(r2 - 0.4, 1.09) + 0.4;
		}
		float yaw2 = -yaw + M_PI + cam1_offset_yaw;
		u1 = cam1_horizon_r * r2 / cam_aspect_ratio * cos(yaw2) + 0.5 + cam1_offset_x;
		v1 = cam1_horizon_r * r2 * sin(yaw2) + 0.5 + cam1_offset_y;
	}
}
