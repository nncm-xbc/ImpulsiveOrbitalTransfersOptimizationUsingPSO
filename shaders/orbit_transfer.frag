#version 330 core
out vec4 FragColor;

uniform vec3 color;

void main() {
    FragColor = vec4(color, 1.0);

    float dashSize = 10.0;
    float gapSize = 5.0;
    if (fract(gl_FragCoord.x / (dashSize + gapSize)) > dashSize / (dashSize + gapSize))
        discard;
}
