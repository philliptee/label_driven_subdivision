#version 410 core

out vec4 color;

flat in int id;

void main()
{
    color = vec4(
        ((id & 0x000000FF) >> 0)/255.0f,
        ((id & 0x0000FF00) >> 8)/255.0f,
        ((id & 0x00FF0000) >> 16)/255.0f,
         1.0f);
}