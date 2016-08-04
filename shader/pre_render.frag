varying vec4 position;

uniform mat4 unif_matrix;
uniform sampler2D tex;

const float M_PI = 3.1415926535;
const float image_r = 0.85;
const vec2 center1 = vec2(0.50, 0.50);

void main(void) {
        float u = 0.0;
        float v = 0.0;
        vec4 pos = unif_matrix * position;
        float roll = asin(pos.y);
        float yaw = atan(pos.x, pos.z);
        float r = (M_PI / 2.0 - roll) / M_PI;
        float yaw2 = yaw + M_PI;
        u = image_r * r * cos(yaw2) + center1.x;
        v = image_r * r * sin(yaw2) + center1.y;
        //gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
        gl_FragColor = texture2D(tex, vec2(u, v));
}
