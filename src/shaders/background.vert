#version 330 core

layout (location=0) in vec2 in_texcoord_0;
layout (location=1) in vec2 in_position;



out vec2 texcoord;

void main() {
    texcoord=in_texcoord_0;
    gl_Position=vec4(in_position,0.0,1.0);
}