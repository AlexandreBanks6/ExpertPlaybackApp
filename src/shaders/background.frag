#version 330 core

in vec2 fragment_uv;
layout (location=0) out vec4 fragment_color;

uniform sampler2D texture0;

void main(){
    fragment_color=texture(texture0,fragment_uv);
}