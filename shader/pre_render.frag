varying vec2 tcoord;
uniform mat4 unif_matrix;
uniform sampler2D tex;

const float M_PI = 3.1415926535;
const float image_r = 0.85;
const vec2 center1 = vec2(0.50, 0.50);
float k = 1.0;
float step = 1.0 / 1024.0;

void main(void) {
        float fov = 45.0;
        float u_factor = image_r;
        float v_factor = image_r;
        float u = 0.0;
        float v = 0.0;
        vec4 pos = vec4(0.0, 0.0, 0.0, 1.0);
        float z = 0.75;
        float len = length(vec2(tcoord.x, z));
        float roll_orig = atan(tcoord.y, len);
        float yaw_orig = -atan(tcoord.x, z);
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
                float r = (roll + M_PI / 2.0) / M_PI;
                float yaw2 = yaw + M_PI;
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
        //else
        {
				vec4 fc = texture2D(tex, vec2(u, v)) * (1.0 + 4.0 * k);
		
				//fc -= texture2D(tex, vec2(u - 1.0*step, v - 1.0*step)) * k;
				fc -= texture2D(tex, vec2(u - 1.0*step, v)) * k;
				//fc -= texture2D(tex, vec2(u - 1.0*step, v + 1.0*step)) * k;
		
				fc -= texture2D(tex, vec2(u, v - 1.0*step)) * k;
				fc -= texture2D(tex, vec2(u, v + 1.0*step)) * k;
		
				//fc -= texture2D(tex, vec2(u + 1.0*step, v - 1.0*step)) * k;
				fc -= texture2D(tex, vec2(u + 1.0*step, v)) * k;
				//fc -= texture2D(tex, vec2(u + 1.0*step, v + 1.0*step)) * k;
                gl_FragColor = fc;
        }
}
