#if (__VERSION__ > 120)
#define IN in
#define OUT out
#else
#define IN attribute
#define OUT varying
#endif // __VERSION
precision mediump float;
IN vec4 vPosition;
OUT vec2 tcoord;

void main(void) {
	vec4 pos = vPosition;
	tcoord = pos.xy;
	pos.xy = pos.xy * vec2(2, 2) + vec2(-1, -1);
	gl_Position = pos;
}