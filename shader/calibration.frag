varying vec2 tcoord;
uniform sampler2D cam0_texture;

void main(void) {
	
	gl_FragColor = texture2D(cam0_texture, vec2(tcoord.x, 1.0 - tcoord.y));
}
