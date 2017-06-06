varying vec2 tcoord;
uniform sampler2D tex;
uniform float tex_scalex;

void main(void) {
	gl_FragColor = texture2D(tex, vec2(tcoord.x * tex_scalex, 1.0 - tcoord.y));
}
