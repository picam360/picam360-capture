attribute vec4 vPosition;

uniform mat4 unif_matrix;
uniform sampler2D tex;

const float M_PI = 3.1415926535;
const float image_r = 0.85;
const vec2 center1 = vec2(0.50, 0.50);

void main(void) {
        float u = 0.0;
        float v = 0.0;
        vec4 pos = unif_matrix * vPosition;
        float roll = asin(pos.y);
        float yaw = atan(pos.x, pos.z);
        float r = (roll + M_PI / 2.0) / M_PI;
        float yaw2 = yaw + M_PI;
        u = image_r * r * cos(yaw2) + center1.x;
        v = image_r * r * sin(yaw2) + center1.y;
        if (u <= 0.0 || u > 1.0 || v <= 0.0 || v > 1.0) {
                u = 0.0;
                v = 0.0;
        }
        //gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
        gl_FragColor = texture2D(tex, vec2(u, v));
}
