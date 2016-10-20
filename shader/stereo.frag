varying vec2 tcoord;
uniform sampler2D tex;

void main(void) {
	gl_FragColor = texture2D(tex, vec2(tcoord.x, 1.0 - tcoord.y));
}
