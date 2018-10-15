precision mediump float;
uniform sampler2D texture_uniform;
varying vec2 v_frag_uv;
varying vec4 v_color;
void main() {
	gl_FragColor = vec4(v_color.xyz, v_color.a * texture2D(texture_uniform, v_frag_uv).a);
}