uniform mat4 unif_matrix;
uniform mat4 unif_matrix_1;
uniform sampler2D cam0_texture;
uniform sampler2D cam1_texture;
uniform float color_offset;
uniform float overlap;

const float M_PI = 3.1415926535;

varying float r0;
varying float r1;
varying float u0;
varying float v0;
varying float u1;
varying float v1;

void main(void) {
	vec4 fc0;
	vec4 fc1;
	if (r0 < 0.5 + overlap) {
		if (u0 <= 0.0 || u0 > 1.0 || v0 <= 0.0 || v0 > 1.0) {
			fc0 = vec4(0.0, 0.0, 0.0, 1.0);
		} else {
			vec4 fc = texture2D(cam0_texture, vec2(u0, v0));
			fc = (fc - color_offset) * color_factor;

			fc0 = fc;
		}
	}
	
	if (r1 > 0.5 - overlap) {
		if (u1 <= 0.0 || u1 > 1.0 || v1 <= 0.0 || v1 > 1.0) {
			fc1 = vec4(0.0, 0.0, 0.0, 1.0);
		} else {
			vec4 fc = texture2D(cam1_texture, vec2(u1, v1));
			fc = (fc - color_offset) * color_factor;

			fc1 = fc;
		}
	}
	if (r0 < 0.5 - overlap) {
		gl_FragColor = fc0;
	} else if (r0 < 0.5 + overlap) {
		gl_FragColor = (fc0 * ((0.5 + overlap) - r0) + fc1 * (r0 - (0.5 - overlap))) / (overlap * 2.0);
	} else {
		gl_FragColor = fc1;
	}
}
