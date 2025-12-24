#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in float aIntensity;
layout(location = 2) in float aClassification;

out float vHeight;
out float vIntensity;
flat out float vClassification;

uniform mat4 uViewProjection;

void main()
{
    vec4 worldPos = vec4(aPosition * 0.01, 1.0);
    vHeight = worldPos.z;
    vIntensity = aIntensity;
    vClassification = aClassification;
    gl_PointSize = 3.0;
    gl_Position = uViewProjection * worldPos;
}
