#version 330 core
in float vHeight;
in float vIntensity;
flat in float vClassification;
out vec4 FragColor;

uniform float uMinHeight;
uniform float uMaxHeight;
uniform float uClipValue;
uniform int uColorMode;
uniform int uAlphaMode;
uniform vec3 uGroundColor;
uniform vec3 uNonGroundColor;
uniform float uCommonAlpha;
uniform float uGroundPlaneAlpha;
uniform float uNonGroundPlaneAlpha;
uniform vec3 uZoneColors[5];
uniform bool uUseZoneColors;

vec3 evaluateHeightColor(float normalizedHeight)
{
    vec3 cool = vec3(0.1, 0.2, 0.9);
    vec3 warm = vec3(0.9, 0.3, 0.0);
    return mix(cool, warm, normalizedHeight);
}

vec3 evaluateIntensityColor(float normalizedIntensity)
{
    vec3 cool = vec3(0.1, 0.9, 0.35);
    vec3 warm = vec3(0.9, 0.3, 0.0);
    return mix(cool, warm, normalizedIntensity);
}

float normalizeHeight(float height)
{
    float range = max(uMaxHeight - uMinHeight, 0.001);
    return clamp((height - uMinHeight) / range, 0.0, 1.0);
}

float normalizeIntensity(float intensity)
{
    return clamp(intensity / max(uClipValue, 0.001), 0.0, 1.0);
}

float computeAlpha(float normalizedIntensity, int colorMode, bool useZoneColors)
{
    float alpha = uCommonAlpha;
    if (uAlphaMode == 1)
    {
        alpha = max(normalizedIntensity, 0.1);
    }

    if (colorMode == 0 && uAlphaMode == 0 && !useZoneColors)
    {
        alpha = mix(uGroundPlaneAlpha, uNonGroundPlaneAlpha, vClassification);
    }

    return alpha;
}

void main()
{
    vec3 color = vec3(1.0);
    float normalizedIntensity = normalizeIntensity(vIntensity);

    if (uColorMode == 0)
    {
        if (uUseZoneColors)
        {
            int zone = clamp(int(floor(vClassification + 0.5)), 0, 4);
            color = uZoneColors[zone];
        }
        else
        {
            color = mix(uGroundColor, uNonGroundColor, vClassification);
        }
    }
    else if (uColorMode == 1)
    {
        color = evaluateHeightColor(normalizeHeight(vHeight));
    }
    else if (uColorMode == 2)
    {
        color = evaluateIntensityColor(normalizedIntensity);
    }

    float alpha = computeAlpha(normalizedIntensity, uColorMode, uUseZoneColors);
    FragColor = vec4(color, alpha);
}
