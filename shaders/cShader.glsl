#version 460 core

// -------------------------------------------------
//                  GLOBAL PARAMETERS 
// -------------------------------------------------

#define MAX_SPHERES
#define MAX_BOUNCES 3
#define SAMPLES_PER_PIXEL 10
#define INFINITY (1./0.)
#define NEG_INFINITY (-INFINITY)
#define EPSILON 0.0001
#define PI 3.14159265359

ivec2 pixelCoords;
ivec2 dims;

// -------------------------------------------------
//                      TYPES 
// -------------------------------------------------
#define LAMBERTIAN 0
#define METAL 1
#define DIELECTRIC 2
#define EMISSIVE 3

#define SPHERE 0
#define TRIANGLE 1
#define LIGHT 2

struct Material
{
    vec4 albedo; 
    int type;
    float fuzz;
    float refIdx;
    float emissionPower;          
};

struct Sphere
{
    vec4 center;
    float radius;
    int matIdx;
};

struct Triangle
{
    int a;
    int b;
    int c;
    vec4 center;
    vec4 normal;
    int matIdx;      
};

struct PrimitiveInfo
{
    int idx;
    int type;       
};

struct Node
{
    vec4 minB;
    vec4 maxB;
    int count;
    int content;
};

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
    Material material;       
    Ray scatterRay;
    vec3 hitPoint;
    vec3 normal;
    Interval rayT;
    float t;
    bool hit;
    bool frontFace;
};

struct Camera
{
    vec3 position;
    vec3 front;
    vec3 right;
    vec3 up;
};

// -------------------------------------------------
//                  UNIFORMS & SSBOs
// -------------------------------------------------

layout(local_size_x = 4, local_size_y = 8, local_size_z=1) in;

layout(rgba32f, binding = 0) uniform image2D currentFrameBuffer;
layout(rgba32f, binding = 1) uniform image2D hitBuffer;
layout(rgba32f, binding = 2) uniform image2D accumulationBuffer;

layout(std430, binding = 3) readonly buffer SpheresData    { Sphere spheres[]; };

layout(std430, binding = 4) readonly buffer VerticesData   { float vertices[]; };

layout(std430, binding = 5) readonly buffer TrianglesData  { Triangle triangles[]; };

layout(std430, binding = 6) readonly buffer MaterialsData  { Material materials[]; };

layout(std430, binding = 7) readonly buffer ObjectIDs      { PrimitiveInfo objectIDs[]; };

layout(std430, binding = 8) readonly buffer ObjectsBVH     { Node objectsBVH[]; };

layout(location = 1) uniform float uTime;
layout(location = 2) uniform int   uFrameCount;
layout(location = 3) uniform vec3  position;
layout(location = 4) uniform ivec4 sceneInfo;
layout(location = 5) uniform mat4  invView;
layout(location = 6) uniform mat4  invProj;

// -------------------------------------------------
//                   Random Utils 
// -------------------------------------------------

// Global seed for generating random numbers
uint gSeed = 0;

// Generates random number between [0,1)
float randomFloat(inout uint seed)
{
    // PCG hash
    uint state = seed * 747796405U + 2891336453U;
    uint word = ((state >> ((state >> 28U) + 4U)) ^ state) * 277803737U;
    seed = (word >> 22U) ^ word;
    
    return seed/float(0xffffffffU);
}

// Generates random number between [min, max)
float randomFloatRange(inout uint seed, in float min, in float max)
{
    return min + (max-min)*randomFloat(seed);
}

// -------------------------------------------------
//                    Vec Utils 
// -------------------------------------------------

// Generates random vector whose components are between [0,1)
vec3 randomVec3(inout uint seed)
{
    return vec3(
            randomFloat(seed),
            randomFloat(seed),
            randomFloat(seed));
}

// Generates random vector whose components are between [min,max)
vec3 randomVec3Range(inout uint seed, in float min, in float max)
{
    return vec3(
            randomFloatRange(seed, min, max),
            randomFloatRange(seed, min, max),
            randomFloatRange(seed, min, max));
}

vec3 randomVec3InUnitDisk(inout uint seed)
{
    vec3 v;
    while (true)
    {
        v = vec3(randomFloatRange(seed, -1, 1), randomFloatRange(seed, -1, 1), 0);
        if (dot(v,v) < 1){return v;}
    }
}

vec3 randomVec3InUnitSphere(inout uint seed)
{
    vec3 v;
    while (true)
    {
        v = randomVec3Range(seed, -1, 1);
        if (dot(v,v) < 1){return v;}
    } 
}

vec3 randomUnitVec3(inout uint seed)
{
    return normalize(randomVec3InUnitSphere(seed));
}

bool nearZero(in vec3 v)
{
    return (abs(v.x) < EPSILON && abs(v.y) < EPSILON && abs(v.z) < EPSILON);
}

vec3 vertex(in int vIdx)
{
    return vec3(vertices[vIdx*3], vertices[vIdx*3+1], vertices[vIdx*3+2]);
}

// -------------------------------------------------
//                   Ray Functions
// -------------------------------------------------

Ray ray(in vec3 origin, in vec3 direction) 
{   
    Ray r;
    r.orig = origin;
    r.dir = direction;
    return r;
}

vec3 pointAt(in Ray r, in float t)
{
    return r.orig + t*r.dir;
}


// Determines the probability of reflection using Schlik's approximation
float reflectance(in float cosine, in float refIdx)
{
    float r0 = (1 - refIdx)/(1 + refIdx);
    r0 = r0 * r0;
    return r0 + (1 - r0) * pow(1 - cosine, 5);
}

// -------------------------------------------------
//                 Interval Functions 
// -------------------------------------------------

Interval interval()
{
    Interval i;
    i.min = INFINITY;
    i.max = NEG_INFINITY;
    return i;
}

Interval interval(in float min_, in float max_)
{
    Interval i;
    i.min = min_;
    i.max = max_;
    return i;
}

bool contains(in Interval i, in float x){return i.min <= x && x <= i.max;}

bool surrounds(in Interval i, in float x){return i.min < x && x < i.max;}

float size(in Interval i){return i.max-i.min;}

Interval expand(in float delta, in Interval i)
{
    float padding = delta/2;
    return interval(i.min - padding, i.max + padding);
}

// -------------------------------------------------
//                   Hit Functions
// -------------------------------------------------

void setFaceNormal(inout HitRecord rec, in Ray r, in vec3 outwardNormal)
{    
    rec.frontFace = dot(r.dir, outwardNormal) < 0;
    if (rec.frontFace) rec.normal = outwardNormal;
    else rec.normal = -outwardNormal;
}

bool hitSphere(in Sphere s, in Ray r, in Interval rayT, inout HitRecord rec)
{
    // Ray Intersection
    const vec3  oc = r.orig - s.center.xyz;
    const float a = dot(r.dir, r.dir); // GLSL has no squared length function, that is why dot product is used.
    const float half_b = dot(oc, r.dir);
    const float c = dot(oc, oc) - s.radius*s.radius;
    
    const float disc = half_b*half_b - a*c;
    
    if (disc < 0) return false;
    const float sqrtDisc = sqrt(disc);

    // find the nearest root that lies in the acceptable range
    float root = (-half_b - sqrtDisc)/a;
    if (!contains(rayT, root))
    {
        root = (-half_b + sqrtDisc)/a;
        if (!contains(rayT, root)) return false;
    }

    rec.t = root;
    rec.hitPoint = pointAt(r, rec.t);
    const vec3 outwardNormal = (rec.hitPoint - s.center.xyz)/s.radius; // By dividing all componentes by the sphere radius the vector is normalized
    setFaceNormal(rec, r, outwardNormal);
    rec.material = materials[s.matIdx];   

    return true;
}

bool hitTriangle(in Triangle tri, in Ray r, in Interval rayT, inout HitRecord rec)
{
    const vec3 a = vertex(tri.a);
    const vec3 b = vertex(tri.b);
    const vec3 c = vertex(tri.c);
    const vec3 edge1 = b - a;
    const vec3 edge2 = c - a;
    const vec3 rayCrossEdge2 = cross(r.dir, edge2);
    const float det = dot(edge1, rayCrossEdge2);

    if (abs(det)< EPSILON) return false;

    const float invDet = 1.0/det;
    
    const vec3 s = r.orig - a;
    const float u = invDet * dot(s, rayCrossEdge2); 
    if (u < 0 || u > 1) return false;   

    const vec3 sCrossEdge1 = cross(s, edge1);
    const float v = invDet * dot(r.dir, sCrossEdge1);
    if (v < 0 || u + v > 1) return false;

    // At this stage we can compute t to find out where the intersection point is on the line.
    const float t = invDet * dot(edge2, sCrossEdge1);
    if (t < rayT.min  || rayT.max < t ) return false;

    rec.t = t;
    rec.hitPoint = pointAt(r, rec.t);
    const vec3 outwardNormal = normalize(cross(edge1,edge2));
    setFaceNormal(rec, r, outwardNormal);
    rec.material = materials[tri.matIdx];
    return true;
}

float hitBB(in Ray ray, in Node node, in float nearestHit) {

    const vec3 tMin = (node.minB.xyz - ray.orig) / ray.dir;
    const vec3 tMax = (node.maxB.xyz - ray.orig) / ray.dir;
    const vec3 t1 = min(tMin, tMax);
    const vec3 t2 = max(tMin, tMax);
    const float tNear = max(max(t1.x, t1.y), t1.z);
    const float tFar = min(min(t2.x, t2.y), t2.z);
    if (tNear <= tFar && tFar > 0 && tNear < nearestHit) return tNear; 
    else return 999999999;
}

// -------------------------------------------------
//                Material Scatter Ray
// -------------------------------------------------

// Computes the scatter ray depending on the material registered in the hit record
bool scatter(inout Ray r, in HitRecord rec)
{
    if (rec.material.type == LAMBERTIAN)
    {
        vec3 scatterDir = rec.normal + randomUnitVec3(gSeed);
        scatterDir = nearZero(scatterDir) ? rec.normal : scatterDir;
        r = ray(rec.hitPoint, scatterDir);
        return true;
    }
    if (rec.material.type == METAL)
    {
        vec3 reflected = reflect(normalize(r.dir), rec.normal);
        r = ray(rec.hitPoint, reflected + rec.material.fuzz * randomUnitVec3(gSeed));
        return dot(r.dir, rec.normal) > 0;
    }
    if (rec.material.type == DIELECTRIC)
    {
        float refractionRatio = rec.frontFace ? (1.0/rec.material.refIdx) : rec.material.refIdx;

        vec3 unitDir = normalize(r.dir);
        float cosTheta = min(dot(-1*unitDir, rec.normal), 1.0);
        float sinTheta = sqrt(1.0 - cosTheta*cosTheta);

        bool cannotRefract = (refractionRatio * sinTheta) > 1.0;
        vec3 dir;

        if (cannotRefract || (reflectance(cosTheta, refractionRatio) > randomFloat(gSeed)))
        {    
            dir = reflect(unitDir, rec.normal);
        }
        else {dir = refract(unitDir, rec.normal, refractionRatio);}

        r = ray(rec.hitPoint, dir);
        return true;
    }
    if (rec.material.type == EMISSIVE) return false;
}

HitRecord traceRay(in Ray r)
{
    HitRecord rec;
    rec.hit = false;
    rec.scatterRay = r;
    float nearestHit = 9999999;
    
    Node node = objectsBVH[0];
    Node stack[30];
    int stackPos = 0;

    while(true) {
        const int objectCount = node.count;
        const int content = node.content;
        
        if (objectCount > 0) {
            
            for(int i = 0; i < objectCount; i++) {
                PrimitiveInfo objInfo = objectIDs[i + content];
                
                if (objInfo.type == SPHERE && hitSphere(spheres[objInfo.idx], r, interval(0.001, nearestHit), rec)){
                    rec.hit = true;
                    nearestHit = rec.t;
                }
                else if (hitTriangle(triangles[objInfo.idx], r, interval(0.001, nearestHit), rec)){
                    rec.hit = true;
                    nearestHit = rec.t;
                }
            }
            
            if (stackPos == 0) break;
            
            else {
                node = stack[--stackPos];
                continue;
            }
        } 
        else {
            Node leftChild = objectsBVH[content];
            Node rightChild = objectsBVH[content + 1];

            float dist1 = hitBB(r, leftChild, nearestHit);
            float dist2 = hitBB(r, rightChild, nearestHit);

            if (dist1 > dist2) {
                Node temp = leftChild;
                leftChild = rightChild;
                rightChild = temp;

                float tempDist = dist1;
                dist1 = dist2;
                dist2 = tempDist;
            }

            if (dist1 > nearestHit) {
                if (stackPos == 0) {
                    break;
                }
                else {
                    node = stack[--stackPos];
                }
            }
            else {
                node = leftChild;
                if (dist2 <= nearestHit) {
                    stack[stackPos++] = rightChild;
                }
            }
        }
    }
    
    return rec;
}

vec3 traceLightRay(in vec3 hitPoint, in vec3 position)
{
    vec3 light = vec3(0.0);
    for (int i = sceneInfo[0] - sceneInfo[3]; i < sceneInfo[0]; i++)
    {        
        HitRecord rec;
        vec3 w = spheres[sceneInfo[0]-1].center.xyz - hitPoint; // vector pointing towards the light
        float distance = length(w);
        normalize(w);
        vec3 h = normalize(position - hitPoint + w);
        float attenuation = 1.0/distance;
        rec.scatterRay = ray(hitPoint, w);
    
        Node node = objectsBVH[0];
        Node stack[30];
        int stackPos = 0;
        rec = traceRay(rec.scatterRay);

        if (rec.material.type == EMISSIVE)
            light = min(light + rec.material.albedo.xyz * attenuation, vec3(1.0));
            // light = min(light + rec.material.albedo.xyz * rec.material.emissionPower * attenuation, vec3(1.0));
    }        
    return light;
}

vec3 rayColor(in Ray r)
{
    vec3 light = vec3(0.0);
    vec3 contribution = vec3(1.0);
    
    HitRecord rec;
    rec.scatterRay = r;
    vec3 viewPoint = r.orig;

    for(int bounce = 0; bounce < MAX_BOUNCES; bounce++)
    {
        rec = traceRay(rec.scatterRay);

        if(!rec.hit)
        {
            const vec3 skyColor = vec3(0.1, 0.1, 0.1);
            light += skyColor * contribution;
            break;
        }
        else if(scatter(rec.scatterRay, rec))
        {
            contribution *= rec.material.albedo.xyz;
            // light += traceLightRay(rec.hitPoint, viewPoint) * contribution;
            viewPoint = rec.hitPoint;
        } 
        else 
        { 
            light += rec.material.albedo.xyz * rec.material.emissionPower;
            break;
        }

        if (bounce == 0)
        imageStore(hitBuffer, pixelCoords, vec4(rec.hitPoint, 0));
    }
    return light;
}

// vec3 rayColor(in Ray r)
// {
//     vec3 light = vec3(0.0);
//     vec3 contribution = vec3(1.0);
    
//     HitRecord rec;
//     rec.scatterRay = r;

//     for(int bounce = 0; bounce < MAX_BOUNCES; bounce++)
//     {
//         rec = traceRay(rec.scatterRay);

//         if(!rec.hit)
//         {
//             vec3 skyColor = vec3(0.6, 0.7, 0.9);
//             // light += skyColor * contribution;
//             break;
//         }
//         else if(scatter(rec.scatterRay, rec))
//         {
//             contribution *= rec.material.albedo.xyz;
//             light += traceLightRay(rec.hitPoint) * contribution;
//         } 
//         else 
//         { 
//             light += rec.material.albedo.xyz * rec.material.emissionPower;
//             break;
//         }
//     }
//     return light;
    
// }


// -------------------------------------------------
//                   Main Function
// -------------------------------------------------

void main()
{    
    // Pixel Coords
    pixelCoords = ivec2(gl_GlobalInvocationID.xy);
    dims = imageSize(currentFrameBuffer);

    gSeed = floatBitsToUint(pixelCoords.x * pixelCoords.y * uTime);
    
    const float x = float(pixelCoords.x * 2 - dims.x)/dims.x;
    const float y = float(pixelCoords.y * 2 - dims.y)/dims.y;

    // const vec3 direction = front + x*right + y*up*dims.y/dims.x;
    const vec4 target = invProj * vec4(x, y, 1, 1);
    const vec3 direction = vec3(invView  * vec4(normalize(target.xyz), 0));
    
    Ray r = ray(position, direction);
    vec3 pixelColor = rayColor(r);
    
    // Gamma correction
    pixelColor = sqrt(pixelColor);

    vec3 accumColor;
    if (uFrameCount > 1){        
        accumColor = imageLoad(accumulationBuffer, pixelCoords).rgb;
        accumColor += pixelColor;
        pixelColor = accumColor/uFrameCount;
    } 
    else accumColor = pixelColor;

    imageStore(currentFrameBuffer, pixelCoords, vec4(pixelColor,1.0));
    imageStore(accumulationBuffer, pixelCoords, vec4(accumColor, 1.0));
}


// -------------------------------------------------
//                     Sampling
// -------------------------------------------------

// Returns a random point in the square surrounding a pixel at the origin 
// vec3 pixelSampleSquare(in Camera c)
// {
//     float px = -0.5 + randomFloat(gSeed);
//     float py = -0.5 + randomFloat(gSeed);
//     return px * c.pixelDeltaU + py * c.pixelDeltaV;
// }

// Get a randomly sampled camera ray for the pixel at location i,j
// Ray getRay(in Camera c, in int i, in int j)
// {
//     vec3 pixelCenter = c.pixel00Loc + (i * c.pixelDeltaU) + (j * c.pixelDeltaV);
//     vec3 pixelSample = pixelCenter + pixelSampleSquare(c);

//     vec3 rayDirection = pixelSample - c.lookFrom;
//     return ray(c.lookFrom, rayDirection);
// }
