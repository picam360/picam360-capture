varying vec2 tcoord;
uniform sampler2D logo_texture;
uniform sampler2D cam_texture;
uniform float pixel_size;
uniform int active_cam;
//options start
uniform float sharpness_gain;
uniform float cam_offset_yaw;
uniform float cam_offset_x;
uniform float cam_offset_y;
uniform float cam_horizon_r;
//options end

void main(void) {	
	vec4 fc = texture2D(logo_texture, vec2(tcoord.x, tcoord.y));
	if (fc.g == 0.0) {
		float u = tcoord.x + cam_offset_x;
		float v = tcoord.y + cam_offset_y;
		if (sharpness_gain == 0.0) {
			fc = texture2D(cam_texture, vec2(u, v));
		} else {
			//sharpness
			float gain = sharpness_gain;
			fc = texture2D(cam_texture, vec2(u, v))
					* (1.0 + 4.0 * gain);
			fc -= texture2D(cam_texture, vec2(u - 1.0 * pixel_size, v))
					* gain;
			fc -= texture2D(cam_texture, vec2(u, v - 1.0 * pixel_size))
					* gain;
			fc -= texture2D(cam_texture, vec2(u, v + 1.0 * pixel_size))
					* gain;
			fc -= texture2D(cam_texture, vec2(u + 1.0 * pixel_size, v))
					* gain;
		}
	}
	gl_FragColor = fc;
}
