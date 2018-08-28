#if (__VERSION__ > 120)
# define IN in
# define OUT out
# define texture2D texture
# define gl_FragColor FragColor
layout (location=0) out vec4 FragColor;
#else
# define IN varying
# define OUT varying
#endif // __VERSION
precision mediump float;
IN vec2 tcoord;
uniform sampler2D tex;
uniform float tex_scalex;

void main(void) {
	gl_FragColor = texture2D(tex, vec2(tcoord.x * tex_scalex, 1.0 - tcoord.y));
}
