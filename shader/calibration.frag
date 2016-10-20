varying vec2 tcoord;
uniform sampler2D cam0_texture;
uniform float pixel_size;
//options start
uniform float sharpness_gain;
uniform float cam0_offset_yaw;
uniform float cam0_offset_x;
uniform float cam0_offset_y;
uniform float cam0_horizon_r;
//options end

void main(void) {
	float u = tcoord.x + cam0_offset_x;
	float v = (1.0 - tcoord.y) - cam0_offset_y;//cordinate is different
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
	gl_FragColor = fc;
}
