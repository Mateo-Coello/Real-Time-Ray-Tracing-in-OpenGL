#version 460 core

precision highp float;
precision highp int;

// -------------------------------------------------
//                  UNIFORMS & SSBOs
// -------------------------------------------------

layout(local_size_x = 4, local_size_y = 8, local_size_z=1) in;

layout(rgba32f, binding = 0) uniform image2D currentFrameBuffer;

layout(location = 1) uniform float uTime;
layout(location = 2) uniform int   uFrameCount;
layout(location = 3) uniform vec3  position;
layout(location = 4) uniform mat4  invView;
layout(location = 5) uniform mat4  invProj;
layout(location = 6) uniform vec3  vertexA;
layout(location = 7) uniform vec3  vertexB;
layout(location = 8) uniform vec3  vertexC;
layout(location = 9) uniform vec3  color;

#define EPSILON 0.0000001

struct Ray
{
    vec3 orig;
    vec3 dir;
};    

struct Interval
{
    float min;
    float max;      
};

struct HitRecord
{
    vec3 hitPoint;
    vec3 normal;
    Interval rayT;
    float t;
    bool hit;
    bool frontFace;
};

Ray ray(in vec3 origin, in vec3 direction) 
{   
    Ray r;
    r.orig = origin;
    r.dir = direction;
    return r;
}

vec3 pointAt(in Ray r, in float t){return r.orig + t*r.dir;}

Interval interval(in float min, in float max)
{
    Interval i;
    i.min = min;
    i.max = max;
    return i;
}

bool contains(in Interval i, in float x){return i.min <= x && x <= i.max;}
bool surrounds(in Interval i, in float x){return i.min < x && x < i.max;}

void setFaceNormal(inout HitRecord rec, in Ray r, in vec3 outwardNormal)
{    
    rec.frontFace = dot(r.dir, outwardNormal) < 0;
    if (rec.frontFace) rec.normal = outwardNormal;
    else rec.normal = -outwardNormal;
}

bool hitTriangle(in Ray r, inout HitRecord rec)
{
    vec3 edge1 = vertexB - vertexA;
    vec3 edge2 = vertexC - vertexA;
    vec3 rayCrossEdge2 = cross(r.dir, edge2);
    float det = dot(edge1, rayCrossEdge2);

    if (abs(det) < EPSILON) return false;

    float invDet = 1.0/det;
    
    vec3 s = r.orig - vertexA;
    float u = invDet * dot(s, rayCrossEdge2); 
    if (u < 0 || u > 1) return false;   

    vec3 sCrossEdge1 = cross(s, edge1);
    float v = invDet * dot(r.dir, sCrossEdge1);
    if (v < 0 || u + v > 1) return false;

    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = invDet * dot(edge2, sCrossEdge1);
    if (!surrounds(rec.rayT, t)) return false;

    rec.t = t;
    rec.hitPoint = pointAt(r, rec.t);
    vec3 outwardNormal = normalize(cross(edge1,edge2));
    setFaceNormal(rec, r, outwardNormal);
    return true;
}

// Function to compute barycentric coordinates
vec3 computeBarycentricCoords(vec3 p, vec3 a, vec3 b, vec3 c) {
    vec3 v0 = b - a;
    vec3 v1 = c - a;
    vec3 v2 = p - a;
    float d00 = dot(v0, v0);
    float d01 = dot(v0, v1);
    float d11 = dot(v1, v1);
    float d20 = dot(v2, v0);
    float d21 = dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0 - v - w;
    return vec3(u, v, w);
}

// -------------------------------------------------
//                   Main Function
// -------------------------------------------------

void main()
{    
    // Pixel Coords
    ivec2 pixelCoords = ivec2(gl_GlobalInvocationID.xy);
    ivec2 dims = imageSize(currentFrameBuffer);

    const float x = float(pixelCoords.x * 2 - dims.x)/dims.x;
    const float y = float(pixelCoords.y * 2 - dims.y)/dims.y;

    const vec4 target = invProj * vec4(x, y, 1, 1);
    const vec3 direction = vec3(invView  * vec4(normalize(target.xyz), 0));
    
    Ray r = ray(position, direction);
    HitRecord rec;
    rec.rayT = interval(0, 999999);
    vec3 pixelColor = vec3(0.0);
    if (hitTriangle(r, rec)){
        if (color.r == -1 || color.g == -1 || color.b == -1){
            pixelColor = computeBarycentricCoords(rec.hitPoint, vertexA, vertexB, vertexC);
        } else pixelColor = color;
    }
    // Gamma correction
    pixelColor = pixelColor / (pixelColor + vec3(1.0));
    pixelColor = pow(pixelColor, vec3(1.0/2.2));

    imageStore(currentFrameBuffer, pixelCoords, vec4(pixelColor, 1.0));
}
