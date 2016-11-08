varying vec2 tcoord;
uniform sampler2D logo_texture;
uniform sampler2D cam0_texture;
uniform sampler2D cam1_texture;
uniform float pixel_size;
uniform int active_cam;
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

void main(void) {
	sampler2D cam_texture;
	uniform float cam_offset_yaw;
	uniform float cam_offset_x;
	uniform float cam_offset_y;
	uniform float cam_horizon_r;
	
	if(active_cam == 1) {
		cam_texture = cam1_texture;
		cam_offset_yaw = cam1_offset_yaw;
		cam_offset_x = cam1_offset_x;
		cam_offset_y = cam1_offset_y;
		cam_horizon_r = cam1_horizon_r;
	} else {
		cam_texture = cam0_texture;
		cam_offset_yaw = cam0_offset_yaw;
		cam_offset_x = cam0_offset_x;
		cam_offset_y = cam0_offset_y;
		cam_horizon_r = cam0_horizon_r;
	}
	
	vec4 fc = texture2D(logo_texture, vec2(tcoord.x, tcoord.y));
	if (fc.g == 0.0) {
		float u = tcoord.x + cam_offset_x;
		float v = tcoord.y - cam_offset_y;
		if (sharpness_gain == 0.0) {
			fc = texture2D(cam_texture, vec2(u, v));
		} else {
			//sharpness
			fc = texture2D(cam_texture, vec2(u, v))
					* (1.0 + 4.0 * sharpness_gain);
			fc -= texture2D(cam_texture, vec2(u - 1.0 * pixel_size, v))
					* sharpness_gain;
			fc -= texture2D(cam_texture, vec2(u, v - 1.0 * pixel_size))
					* sharpness_gain;
			fc -= texture2D(cam_texture, vec2(u, v + 1.0 * pixel_size))
					* sharpness_gain;
			fc -= texture2D(cam_texture, vec2(u + 1.0 * pixel_size, v))
					* sharpness_gain;
		}
	}
	gl_FragColor = fc;
}
