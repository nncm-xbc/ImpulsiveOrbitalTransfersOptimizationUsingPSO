#version 330 core
out vec4 FragColor;

uniform vec3 color1 = vec3(0.8, 0.8, 0.2); // First arc color (yellowish)
uniform vec3 color2 = vec3(0.2, 0.8, 0.8); // Second arc color (blueish)
uniform float animationProgress;

// Interpolated from vertex shader
in float progress;
in float arcIdentifier;

void main() {
    // Only draw fragments that are within the current animation progress
    if (progress > animationProgress) {
        discard; // Discard fragments beyond current animation progress
    }
    
    // Calculate fade effect near the animation front
    float fadeEffect = 1.0;
    if (progress > animationProgress - 0.1) {
        fadeEffect = 1.0 - (animationProgress - progress) * 10.0;
    }
    
    // Blend between the two arc colors based on arc identifier
    vec3 finalColor = mix(color1, color2, arcIdentifier);
    
    // Add a glowing effect at the animation front
    if (abs(progress - animationProgress) < 0.02) {
        finalColor = mix(finalColor, vec3(1.0, 1.0, 1.0), 0.7);
    }
    
    FragColor = vec4(finalColor, fadeEffect);
}