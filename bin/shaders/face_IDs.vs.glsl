#version 460 core

uniform mat4 mv_matrix;
uniform mat4 proj_matrix;

layout (location = 0) in vec4 position;
layout (location = 1) in vec3 normal;

flat out int id;

void main(void)
{
    vec4 pos_vs = mv_matrix * position;
    id = gl_DrawID;
    gl_Position = proj_matrix * pos_vs;
}