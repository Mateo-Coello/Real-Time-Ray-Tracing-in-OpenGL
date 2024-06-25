#version 460 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoords;

out vec2 TexCoords;

void main()
{
    TexCoords = (aPos.xy + 1.0) / 2.0;
    gl_Position = vec4(aPos, 1.0);
}