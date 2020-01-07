#if (__VERSION__ > 120)
#define IN in
#define OUT out
#else
#define IN attribute
#define OUT varying
#endif // __VERSION

#define NUM_OF_CAM %NUM_OF_CAM%

precision highp float;

const int STEPNUM = 32;
const float M_PI = 3.1415926535;
const float M_PI_DIV_2 = M_PI / 2.0;
const float M_PI_DIV_4 = M_PI / 4.0;
const float M_SQRT_2 = 1.4142135623;

IN vec4 vPosition; //[0:1]
//options start
uniform float frame_aspect_ratio;

uniform float pixel_size;
uniform float cam_aspect_ratio;
uniform float sharpness_gain;
//angular map params
uniform float r_table[STEPNUM];
uniform float pitch_table[STEPNUM];

uniform mat4 cam_attitude[NUM_OF_CAM];
uniform float cam_offset_x[NUM_OF_CAM];
uniform float cam_offset_y[NUM_OF_CAM];
uniform float cam_horizon_r[NUM_OF_CAM];
uniform float cam_aov[NUM_OF_CAM];
//options end

OUT vec3 cam_uvr[NUM_OF_CAM];
OUT vec3 logo_uvr;

//abs(pos.y) > 1.0 cause strange image 
#define CAM_UVR(i)\
	{\
		vec4 pos = cam_attitude[i] * position;\
		float pitch = pos.y >= 1.0 ? 0.0 : (pos.y <= -1.0 ? 2.0 * M_PI : acos(pos.y));\
		float yaw = atan(pos.z, pos.x);\
		float r = pitch / M_PI;\
		float r2 = sin(M_PI * 180.0 / cam_aov[i] * r) / 2.0;\
		cam_uvr[i][0] = cam_horizon_r[i] * r2 / cam_aspect_ratio * cos(yaw) + 0.5 + cam_offset_x[i];\
		cam_uvr[i][1] = -(cam_horizon_r[i] * r2 * sin(yaw)) + 0.5 + cam_offset_y[i];\
		cam_uvr[i][2] = r;\
	}

float get_y(float x, float x_table[STEPNUM], float y_table[STEPNUM]) {
	int cur = STEPNUM / 2;
	for (int s = STEPNUM / 4; s > 0; s /= 2) {
		cur += (x <= x_table[cur]) ? -s : s;
	}
	if (x <= x_table[cur]) {
		cur--;
	}
	int cur_p1 = cur + 1;
	float k = (y_table[cur_p1] - y_table[cur]) / (x_table[cur_p1] - x_table[cur]);
	return k * (x - x_table[cur]) + y_table[cur];
}

void main(void) {
	vec4 position;
	position.xy = vPosition.xy * vec2(2.0, 2.0) + vec2(-1.0, -1.0); //[-1:1]
	gl_Position.xy = vec2(position.x, position.y * frame_aspect_ratio);
	gl_Position.zw = vec2(1.0, 1.0);

	float r = sqrt(position.x * position.x + position.y * position.y);
	if (r > M_SQRT_2) {
		r = M_SQRT_2;
	}
	float pitch_orig = get_y(r, r_table, pitch_table);
	float roll_orig = atan(position.y, position.x);
	if (r <= M_SQRT_2 && r > 1.0) {
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
	%FOR_CAM_UVR_NUM_OF_CAM%
	{ //logo
		vec4 pos = cam_attitude[0] * position;
		logo_uvr[0] = pos.x / -pos.y * 0.35 + 0.5;
		logo_uvr[1] = pos.z / -pos.y * 0.35 + 0.5;
		logo_uvr[2] = acos(position.y) / M_PI;
	}
}
