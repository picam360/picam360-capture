attribute vec4 vPosition;
varying vec4 position;

void main(void)
{
	position = vPosition;
	gl_Position = vec4(vPosition.x/vPosition.z, vPosition.y/vPosition.z, 1.0, 1.0);
}
