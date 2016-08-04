attribute vec4 vPosition;
varying vec2 tcoord;

void main(void)
{
    vec4 pos = vPosition;
    gl_Position = pos;
    tcoord = pos.xy;
}
