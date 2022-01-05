/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2012-2021 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include "utils/structs.glsl"
#include "utils/sampler2d.glsl"
#include "utils/sampler3d.glsl"

#include "utils/classification.glsl"
#include "utils/compositing.glsl"
#include "utils/depth.glsl"
#include "utils/gradients.glsl"
#include "utils/shading.glsl"
#include "utils/raycastgeometry.glsl"

#include "utils/isosurface.glsl"

uniform VolumeParameters volumeParameters;
uniform sampler3D volume;

uniform sampler2D transferFunction;

uniform ImageParameters entryParameters;
uniform sampler2D entryColor;
uniform sampler2D entryDepth;
uniform sampler2D entryPicking;
uniform sampler2D entryNormal;

uniform ImageParameters exitParameters;
uniform sampler2D exitColor;
uniform sampler2D exitDepth;

uniform ImageParameters bgParameters;
uniform sampler2D bgColor;
uniform sampler2D bgPicking;
uniform sampler2D bgDepth;

uniform ImageParameters outportParameters;

uniform LightParameters lighting;
uniform CameraParameters camera;
uniform VolumeIndicatorParameters positionindicator;
uniform RaycastingParameters raycaster;
uniform IsovalueParameters isovalues;

uniform bool useNormals = false;

uniform int channel;

uniform float lightRadius;

// TODO: Better manage all these defines => Move to uniforms and defines controllable by GUI
#define ERT_THRESHOLD 0.99  // threshold for early ray termination
#define OPAQUE_THRESHOLD 0.99 // Should this be the same as above? => UNIFORM 
#define TRANSLUCENT_THRESHOLD 0.15 // TODO: Make this a uniform in the future => UNIFORM
#define SOFT_SHADOWS_SAMPLES 16 // => UNIFORM
// #define R 1 // => UNIFORM
#define SOFT_SHADOWS_ENABLED // => DEFINE
#define PI 3.14159265358979

#if (!defined(INCLUDE_DVR) && !defined(INCLUDE_ISOSURFACES))
#  define INCLUDE_DVR
#endif

// http://holger.dammertz.org/stuff/notes_HammersleyOnHemisphere.html
float radicalInverse_VdC(uint bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10; // / 0x100000000
}

// TODO: Optimize by passing the inverse of N
// http://holger.dammertz.org/stuff/notes_HammersleyOnHemisphere.html
vec2 hammersley2d(uint i, uint N) {
    return vec2(float(i)/float(N), radicalInverse_VdC(i));
}

// http://holger.dammertz.org/stuff/notes_HammersleyOnHemisphere.html
vec3 hemisphereSample_uniform(float u, float v) {
    float phi = v * 2.0 * PI;
    float cosTheta = 1.0 - u;
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
    return vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

void computeRayTraversalParameters(in vec3 rayStart, in vec3 rayEnd, in float samplingRate,
                                   out vec3 rayDirection,
                                   out float t, out float tIncr, out float tEnd,
                                   out float samples) {
    rayDirection = rayEnd - rayStart;
    float rayLength = length(rayDirection);

    tEnd = rayLength;
    samples = samplingRate * length(rayDirection * volumeParameters.dimensions);
    tIncr = min(tEnd, tEnd / samples);
    samples = ceil(tEnd / tIncr);
    tIncr = tEnd / samples;
    t = 0.5 * tIncr;

    rayDirection /= rayLength;
}

// Assume rayStart is inside volume and rayEnd is outside volume
vec3 rayVolumeIntersection(vec3 rayStart, vec3 rayEnd) {
    vec3 rayDirection = rayEnd - rayStart;
    vec3 tIntersection0 = (vec3(1.0f, 1.0f, 1.0f) - rayStart) / rayDirection;
    vec3 tIntersection1 = (vec3(0.0f, 0.0f, 0.0f) - rayStart) / rayDirection;
    float tIntersection = 2.0f;
    for (int c = 0; c < 3; ++c) {
        if (tIntersection0[c] > 0.0 && tIntersection0[c] < tIntersection) tIntersection = tIntersection0[c];
        if (tIntersection1[c] > 0.0 && tIntersection1[c] < tIntersection) tIntersection = tIntersection1[c];
    }
    return rayStart + tIntersection * rayDirection;
}

// TODO: Give this a better name
// TODO: Determine offset
bool shadowRayTraversal(vec3 rayStart, vec3 rayEnd) {
    vec3 rayDirection;
    float t;
    float tIncr;
    float tEnd;
    float samples;
    computeRayTraversalParameters(rayStart, rayEnd, raycaster.samplingRate, rayDirection, t, tIncr, tEnd, samples);

    float offset = 0.01;
    t += offset;
    while (t < tEnd) {
        vec3 samplePos = rayStart + t * rayDirection;
        vec4 voxel = getNormalizedVoxel(volume, volumeParameters, samplePos);
        vec4 color = APPLY_CHANNEL_CLASSIFICATION(transferFunction, voxel, channel);
        if (color.a > ERT_THRESHOLD) return true;
        t += tIncr;
    }
    return false;
}

vec3 applyNoShadows(LightParameters lighting,
                    vec3 materialAmbientColor, vec3 materialDiffuseColor, vec3 materialSpecularColor,
                    vec3 position, vec3 normal, vec3 toCameraDir) {
    return APPLY_LIGHTING(lighting, materialAmbientColor, materialDiffuseColor, materialSpecularColor,
                          position, normal, toCameraDir);
}

vec3 applyHardShadows(LightParameters lighting, vec3 samplePosition,
                      vec3 materialAmbientColor, vec3 materialDiffuseColor, vec3 materialSpecularColor,
                      vec3 position, vec3 normal, vec3 toCameraDir) {
    vec3 rayStart = samplePosition;
    vec3 rayEnd = (volumeParameters.worldToTexture * vec4(lighting.position, 1.0)).xyz;
    vec3 exitPoint = rayVolumeIntersection(rayStart, rayEnd);
    bool shadowed = shadowRayTraversal(rayStart, exitPoint);

    if (shadowed) return APPLY_AMBIENT_LIGHTING(lighting, materialAmbientColor);
    else return APPLY_LIGHTING(lighting, materialAmbientColor, materialDiffuseColor, materialSpecularColor,
                                    position, normal, toCameraDir);
}

vec3 applySoftShadows(LightParameters lighting, vec3 samplePosition, uint sampleId, uint totalSamples,
                      vec3 materialAmbientColor, vec3 materialDiffuseColor, vec3 materialSpecularColor,
                      vec3 position, vec3 normal, vec3 toCameraDir) {
    float shadowed = 0.0;

#if defined(SOFT_SHADOWS_ENABLED)
    vec3 rayStart = samplePosition;

    vec3 x = vec3(1.0, 0.0, 0.0);
    vec3 Z = normalize(samplePosition - lighting.position);
    if (distance(x, Z) < 0.01) x = vec3(0.0, 1.0, 0.0);
    vec3 Y = -normalize(cross(x, Z));
    vec3 X = cross(Y, Z);
    mat3 basis = mat3(X, Y, Z);

    int shadowedCount = 0;
    for (int i = 0; i < SOFT_SHADOWS_SAMPLES; ++i) {
        vec2 random = hammersley2d(i * totalSamples + sampleId, SOFT_SHADOWS_SAMPLES * totalSamples);
        vec3 uniformHemisphere = hemisphereSample_uniform(random.x, random.y);
        vec3 lightSample = basis * (lightRadius * uniformHemisphere) + lighting.position;
        vec3 rayEnd = (volumeParameters.worldToTexture * vec4(lightSample, 1.0)).xyz;
        vec3 exitPoint = rayVolumeIntersection(rayStart, rayEnd);
        if (shadowRayTraversal(rayStart, exitPoint)) ++shadowedCount;
    }

    shadowed = float(shadowedCount) / SOFT_SHADOWS_SAMPLES;
#endif

    vec3 lit = APPLY_LIGHTING(lighting, materialAmbientColor, materialDiffuseColor, materialSpecularColor,
                              position, normal, toCameraDir);
    vec3 ambient = APPLY_AMBIENT_LIGHTING(lighting, materialAmbientColor);
    vec3 diff = lit - ambient;
    return ambient + (1.0 - shadowed) * diff;
}

// TODO: Define applySoftShadowsInterleaved for being able to compare vs non-interleaved


vec4 rayTraversal(vec3 entryPoint, vec3 exitPoint, vec2 texCoords, float backgroundDepth, vec3 entryNormal) {
    vec4 result = vec4(0.0);

    vec3 rayDirection;
    float t;
    float tIncr;
    float tEnd;
    float samples;
    computeRayTraversalParameters(entryPoint, exitPoint, raycaster.samplingRate, rayDirection, t, tIncr, tEnd, samples);

    float tDepth = -1.0;
    vec4 color;
    vec4 voxel;
    vec3 samplePos;
    vec3 toCameraDir = normalize((volumeParameters.textureToWorld * vec4(entryPoint, 1.0) -
                                  volumeParameters.textureToWorld * vec4(exitPoint, 1.0))
                                     .xyz);

    vec4 backgroundColor = vec4(0);
    float bgTDepth = -1;
#ifdef BACKGROUND_AVAILABLE
    backgroundColor = texture(bgColor, texCoords);
    // convert to raycasting depth
    bgTDepth = tEnd * calculateTValueFromDepthValue(
        camera, backgroundDepth, texture(entryDepth, texCoords).x, texture(exitDepth, texCoords).x);        

    if (bgTDepth < 0) {
        result = backgroundColor;
    }
#endif // BACKGROUND_AVAILABLE

    // used for isosurface computation
    voxel = getNormalizedVoxel(volume, volumeParameters, entryPoint + t * rayDirection);


    bool first = true;
    int i = 1;
    while (t < tEnd) {
        samplePos = entryPoint + t * rayDirection;
        vec4 previousVoxel = voxel;
        voxel = getNormalizedVoxel(volume, volumeParameters, samplePos);

        // check for isosurfaces
#if defined(ISOSURFACE_ENABLED) && defined(INCLUDE_ISOSURFACES)
        // make sure that tIncr has the correct length since drawIsoSurface will modify it
        tIncr = tEnd / samples;
        result = drawIsosurfaces(result, isovalues, voxel, previousVoxel, 
                                 volume, volumeParameters, channel, transferFunction, camera, lighting, 
                                 samplePos, rayDirection, toCameraDir, t, tIncr, tDepth);
#endif // ISOSURFACE_ENABLED

#if defined(BACKGROUND_AVAILABLE)
        result = DRAW_BACKGROUND(result, t, tIncr, backgroundColor, bgTDepth, tDepth);
#endif // BACKGROUND_AVAILABLE

#if defined(PLANES_ENABLED)
        result = DRAW_PLANES(result, samplePos, rayDirection, tIncr, positionindicator, t, tDepth);
#endif // #if defined(PLANES_ENABLED)

#if defined(INCLUDE_DVR)
        color = APPLY_CHANNEL_CLASSIFICATION(transferFunction, voxel, channel);
        if (color.a > 0) {

            vec3 gradient;
            if (first && useNormals) {
                gradient = -entryNormal;
            } else {
                gradient = COMPUTE_GRADIENT_FOR_CHANNEL(voxel, volume, volumeParameters, samplePos, channel);
                gradient = normalize(gradient);
            }
            // World space position
            vec3 worldSpacePosition = (volumeParameters.textureToWorld * vec4(samplePos, 1.0)).xyz;
            // Note that the gradient is reversed since we define the normal of a surface as
            // the direction towards a lower intensity medium (gradient points in the increasing
            // direction)
            if (color.a > ERT_THRESHOLD)
                color.rgb = applyHardShadows(lighting, samplePos, color.rgb, color.rgb, vec3(1.0),
                                             worldSpacePosition, -gradient, toCameraDir);
            else if (color.a > TRANSLUCENT_THRESHOLD)
                color.rgb = applySoftShadows(lighting, samplePos, i, int(samples), color.rgb, color.rgb, vec3(1.0),
                                             worldSpacePosition, -gradient, toCameraDir);
            else
                color.rgb = applyNoShadows(lighting, color.rgb, color.rgb, vec3(1.0),
                                            worldSpacePosition, -gradient, toCameraDir);

            result = APPLY_COMPOSITING(result, color, samplePos, voxel, gradient, camera,
                                       raycaster.isoValue, t, tDepth, tIncr);
        }
#endif // INCLUDE_DVR

        // early ray termination
        if (result.a > ERT_THRESHOLD) {
            t = tEnd;
        } else {
#if defined(ISOSURFACE_ENABLED) && defined(INCLUDE_ISOSURFACES)
            // make sure that tIncr has the correct length since drawIsoSurface will modify it
            tIncr = tEnd / samples;
#endif // ISOSURFACE_ENABLED
            t += tIncr;
        }
        first = false;
        ++i;
    }

    // composite background if lying beyond the last volume sample, which is located at tEnd - tIncr*0.5
    if (bgTDepth > tEnd - tIncr * 0.5) {
        result =
            DRAW_BACKGROUND(result, bgTDepth, tIncr * 0.5, backgroundColor, bgTDepth, tDepth);
    }

    if (tDepth != -1.0) {
        tDepth = calculateDepthValue(camera, tDepth / tEnd, texture(entryDepth, texCoords).x,
                                     texture(exitDepth, texCoords).x);

    } else {
        tDepth = 1.0;
    }

    gl_FragDepth = min(backgroundDepth, tDepth);

    return result;
}

void main() {
    vec2 texCoords = gl_FragCoord.xy * outportParameters.reciprocalDimensions;
    vec3 entryPoint = texture(entryColor, texCoords).rgb;
    vec3 exitPoint = texture(exitColor, texCoords).rgb;

    vec4 color = vec4(0);

    float backgroundDepth = 1;
#ifdef BACKGROUND_AVAILABLE
    color = texture(bgColor, texCoords);
    gl_FragDepth = backgroundDepth = texture(bgDepth, texCoords).x;
    PickingData = texture(bgPicking, texCoords);
#else // BACKGROUND_AVAILABLE
    PickingData = vec4(0);
    if (entryPoint == exitPoint) {
        discard;
    }
#endif // BACKGROUND_AVAILABLE

    vec3 normal = useNormals ? texture(entryNormal, texCoords).xyz : vec3(0,0,0);
    if (entryPoint != exitPoint) {
        color = rayTraversal(entryPoint, exitPoint, texCoords, backgroundDepth, normal);
    }
    FragData0 = color;
}