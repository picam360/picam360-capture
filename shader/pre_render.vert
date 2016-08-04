attribute vec4 vPosition;
varying vec4 position;
uniform mat4 projection_matrix;

void main(void)
{
	position = vPosition;
    gl_Position = projection_matrix * vPosition;
}
