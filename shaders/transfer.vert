#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in float arcID; // 0.0 for first arc, 1.0 for second arc
layout (location = 2) in float vertexProgress; // Progress value (0.0 to 1.0) for this vertex

uniform mat4 viewProjection;
uniform float animationProgress; // 0.0 to 1.0 progress of animation

// Interpolate to fragment shader
out float progress;
out float arcIdentifier;

void main() {
    // Pass through the arc identifier
    arcIdentifier = arcID;
    
    // Calculate vertex progress along total trajectory (for animation)
    progress = arcID * 0.5 + vertexProgress * 0.5;
    
    gl_Position = viewProjection * vec4(aPos, 1.0);
}