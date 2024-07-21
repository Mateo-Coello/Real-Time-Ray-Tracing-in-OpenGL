#version 460 core
out vec4 FragColor;

in vec2 TexCoords;
uniform sampler2D currentFrameBuffer;

void main()
{
    vec3 texCol = texture(currentFrameBuffer, TexCoords).rgb;
    FragColor = vec4(texCol, 1.0);
}

