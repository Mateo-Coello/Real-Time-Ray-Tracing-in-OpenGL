#version 460 core

layout(local_size_x = 4, local_size_y = 8, local_size_z=1) in;

layout(rgba32f, binding = 0) uniform image2D currentFrameBuffer;
layout(rgba32f, binding = 1) uniform image2D hitBuffer;
layout(rgba32f, binding = 3) uniform image2D previousFrameBuffer;

layout(location = 0) uniform int  uFrameCount;
layout(location = 1) uniform mat4 invT;
layout(location = 2) uniform mat4 pView;
layout(location = 3) uniform mat4 pProj;

void main()
{
    const ivec2 pixelCoords = ivec2(gl_GlobalInvocationID.xy);
    const ivec2 dims = imageSize(currentFrameBuffer);
    const float x = float(pixelCoords.x * 2 - dims.x)/dims.x;
    const float y = float(pixelCoords.y * 2 - dims.y)/dims.y;

    if (uFrameCount == 1) {
        vec4 currentPixelColor = imageLoad(currentFrameBuffer, pixelCoords);
        imageStore(previousFrameBuffer, pixelCoords, currentPixelColor);
    }
    else {        
        vec3 neighbors[9];
        neighbors[0] = imageLoad(currentFrameBuffer, pixelCoords + ivec2(-1,-1)).xyz;
        neighbors[1] = imageLoad(currentFrameBuffer, pixelCoords + ivec2(0,-1)).xyz;
        neighbors[2] = imageLoad(currentFrameBuffer, pixelCoords + ivec2(1,-1)).xyz;
        neighbors[3] = imageLoad(currentFrameBuffer, pixelCoords + ivec2(-1,0)).xyz;
        neighbors[4] = imageLoad(currentFrameBuffer, pixelCoords + ivec2(0,0)).xyz;
        neighbors[5] = imageLoad(currentFrameBuffer, pixelCoords + ivec2(1,0)).xyz;
        neighbors[6] = imageLoad(currentFrameBuffer, pixelCoords + ivec2(-1,1)).xyz;
        neighbors[7] = imageLoad(currentFrameBuffer, pixelCoords + ivec2(0,1)).xyz;
        neighbors[8] = imageLoad(currentFrameBuffer, pixelCoords + ivec2(1,1)).xyz;

        vec4 hitPoint = imageLoad(hitBuffer, pixelCoords);
        ivec2 pPixelCoords = ivec2(pProj * pView * invT * hitPoint);

        vec3 nmin = neighbors[0];
        vec3 nmax = neighbors[0];

        for(int i=1;i<9;i++)
        {
            nmin = min(nmin, neighbors[i]);
            nmax = max(nmax, neighbors[i]);
        }

        vec3 prevSample = clamp(imageLoad(previousFrameBuffer, pPixelCoords).xyz, nmin, nmax);

        float blend = 0.8;
        bvec2 a = greaterThan(pPixelCoords, vec2(dims.x, dims.y));
        bvec2 b = lessThan(pPixelCoords, vec2(0.0, 0.0));

        blend = any(bvec2(any(a), any(b))) ? 1.0 : blend;
        vec3 curSample = neighbors[4];
        vec3 approxPixelColor = mix(prevSample, curSample, blend);
        imageStore (currentFrameBuffer, pixelCoords, vec4(approxPixelColor, 1.0));
    }
}