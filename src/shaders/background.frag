#version 330 core

in vec2 texcoord;
out vec4 fragColor;

uniform sampler2D u_texture_0;

void main(){
    fragColor=texture(u_texture_0,texcoord);
}