varying vec2 tcoord;
uniform mat4 unif_matrix;
uniform sampler2D tex;

const float M_PI = 3.1415926535;
const float aspect = 480.0 / 640.0;
const float image_r = 0.85;
const vec2 center1 = vec2(0.50, 0.50);

void main(void) {
        float fov = 45.0;
        float u_factor = aspect*image_r;
        float v_factor = image_r;
        float u = 0.0;
        float v = 0.0;
        float tcoord_x = ((tcoord.x > 0.0)?tcoord.x-1.0:tcoord.x)*2.0 + 1.0;
        vec4 pos = vec4(0.0, 0.0, 0.0, 1.0);
        float z = 0.75;
        float len = length(vec2(tcoord_x, z));
        float roll_orig = atan(tcoord.y, len);
        float yaw_orig = -atan(tcoord_x, z);
        pos.x = cos(roll_orig) * sin(yaw_orig);//yaw starts from y
        pos.y = cos(roll_orig) * cos(yaw_orig);//yaw starts from y
        pos.z = sin(roll_orig);
        pos = unif_matrix * pos;
        float roll = asin(pos.z);
        float yaw = -atan(pos.x, pos.y);//yaw starts from y
        //float roll = roll_orig;
        //float yaw = yaw_orig;
        //if(roll > -0.1 && roll < 0.1) {
        //} else
        //if (roll > 0.0) {
                float r = (M_PI / 2.0 - roll) / M_PI;
                float yaw2 = -yaw + M_PI;
                u = u_factor * r * cos(yaw2) + center1.x;
                v = v_factor * r * sin(yaw2) + center1.y;
                if (u <= 0.0 || u > 1.0 || v <= 0.0 || v > 1.0) {
                        u = 0.0;
                        v = 0.0;
                }
        //}
        //if (u == 0.0 && v == 0.0) {
        //        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        //}
        //else {
                //gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
                gl_FragColor = texture2D(tex, vec2(u, v));
        //}
}
