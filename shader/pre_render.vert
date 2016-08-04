attribute vec4 vPosition;
uniform mat4 projection_matrix;

void main(void)
{
    gl_Position = projection_matrix * vPosition;
}
