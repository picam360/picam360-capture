#if (__VERSION__ > 120)
#define IN in
#define OUT out
#else
#define IN attribute
#define OUT varying
#endif // __VERSION
precision highp float;

const int MAX_NUM_OF_CAM = 3;
const float M_PI = 3.1415926535;
const float M_PI_DIV_2 = M_PI / 2.0;
const float M_PI_DIV_4 = M_PI / 4.0;
const float M_SQRT_2 = 1.4142135623;

IN vec4 vPosition; //[0:1]
//options start
uniform float scale;
uniform float frame_aspect_ratio;

uniform float pixel_size;
uniform float cam_aspect_ratio;
uniform float sharpness_gain;

uniform int num_of_cam;
uniform sampler2D cam_texture[MAX_NUM_OF_CAM];
uniform mat4 cam_attitude[MAX_NUM_OF_CAM];
uniform float cam_offset_x[MAX_NUM_OF_CAM];
uniform float cam_offset_y[MAX_NUM_OF_CAM];
uniform float cam_horizon_r[MAX_NUM_OF_CAM];
uniform float cam_aov[MAX_NUM_OF_CAM];
//options end

OUT vec3 cam_uvr[MAX_NUM_OF_CAM];
OUT vec3 logo_uvr;

void main(void) {
	vec4 position = vPosition;
	gl_Position = vec4(vPosition.x / vPosition.z * scale, vPosition.y / vPosition.z * scale * frame_aspect_ratio, 1.0, 1.0);

	//if(num_of_cam >= 1)
	{ //cam0
		const int i = 0;
		vec4 pos = cam_attitude[i] * position;
		float pitch = asin(pos.y);
		float yaw = atan(pos.x, pos.z);

		float r = (M_PI / 2.0 - pitch) / M_PI;
		float r2 = sin(M_PI * 180.0 / cam_aov[i] * r) / 2.0;
		cam_uvr[i][0] = cam_horizon_r[i] / cam_aspect_ratio * r2 * cos(yaw) + 0.5 + cam_offset_x[i];
		cam_uvr[i][1] = cam_horizon_r[i] * r2 * sin(yaw) + 0.5 + cam_offset_y[i];
		cam_uvr[i][2] = r;
	}
	{ //logo
		vec4 pos = cam_attitude[0] * position;
		logo_uvr[0] = pos.x / -pos.y * 0.35 + 0.5;
		logo_uvr[1] = pos.z / -pos.y * 0.35 + 0.5;
	}
}