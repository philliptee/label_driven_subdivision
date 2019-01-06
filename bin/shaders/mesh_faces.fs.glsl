#version 460 core

 layout(std430, binding = 3) buffer shader_data
 {
     int data_SSBO[];
 };

in VS_OUT
{
    vec3 normal;
    vec3 view;
} fs_in;

out vec4 color;
flat in int id;

vec4 plain_color = vec4(0.4, 0.4, 0.8, 1);
vec4 yellow_highlight = vec4(0.1, 0.0, 0.0, 0.0);

vec4 ambientColor = vec4(1);

struct LightSource {
    vec4 position;
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
};

LightSource s = LightSource(
vec4(0.5, 0.2, 1.0, 0.0 ),
vec4( 0.1, 0.1, 0.1, 1.0 ),
vec4(0.7, 0.7, 0.7, 1.0 ),
vec4(0.8, 0.8, 0.8, 1.0 )
);
LightSource s2 = LightSource(
vec4(-0.8,  0.4, -1.0, 0.0 ),
vec4( 0.0, 0.0, 0.0, 1.0 ),
vec4(0.5,  0.5, 0.5, 1.0 ),
vec4(0.8,  0.8, 0.8, 1.0 )
);

LightSource light_arr[2] = LightSource[2](s, s2);

vec4 lighting(vec4 diffuse, vec3 Peye, vec3 Neye) {
    vec4 color_shad = yellow_highlight*data_SSBO[id];
    for (int i = 0; i < 2; ++i) {

        vec4 Plight = light_arr[i].position;

        vec3 l = (Plight.w == 0.0)
                    ? normalize(Plight.xyz) : normalize(Plight.xyz - Peye);

        vec3 n = normalize(Neye);
        vec3 h = normalize(l + vec3(0,0,1));    // directional viewer

        float d = max(0.0, dot(n, l));
        float s = pow(max(0.0, dot(n, h)), 500.0f);

        color_shad += light_arr[i].ambient * ambientColor
            + d * light_arr[i].diffuse * diffuse
            + s * light_arr[i].specular;
    }
    color_shad.a = 1;
    return color_shad;
}

void main(void)
{
    vec3 N = (gl_FrontFacing ? fs_in.normal : -fs_in.normal);
    color = lighting(plain_color, fs_in.view, N);
}
