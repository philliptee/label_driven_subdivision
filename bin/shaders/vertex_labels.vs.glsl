#version 460 core

uniform mat4 mv_matrix;
uniform mat4 proj_matrix;

layout (location = 2) in vec4 position;

void main(void)
{
    vec4 pos_vs = mv_matrix * position;
    gl_Position = proj_matrix * pos_vs;
}

