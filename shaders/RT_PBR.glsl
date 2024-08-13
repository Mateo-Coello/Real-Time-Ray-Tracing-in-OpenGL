#version 460 core

precision highp float;
precision highp int;

// -------------------------------------------------
//                  GLOBAL PARAMETERS 
// -------------------------------------------------

#define MAX_BOUNCES 4
#define INFINITY (1./0.)
#define NEG_INFINITY (-INFINITY)
#define EPSILON 0.0000001
#define PI 3.14159265359

ivec2 pixelCoords;
ivec2 dims;

// -------------------------------------------------
//                      TYPES 
// -------------------------------------------------

#define SPHERE 0
#define TRIANGLE 1
#define LIGHT 2

struct Material
{
    vec4 albedo;
    vec4 specular;
    vec4 emission;
    float roughness;
    float metalness;
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
    int splitAxis;
    int nPrimitives;
    int primitiveOffset;
};

struct Ray
{
    vec3 orig;
    vec3 dir;
    vec3 energy;    
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
    vec3 irradiance;
    Interval rayT;
    float t;
    bool hit;
    bool frontFace;
};

// -------------------------------------------------
//                  UNIFORMS & SSBOs
// -------------------------------------------------

layout(local_size_x = 16, local_size_y = 16, local_size_z=1) in;

layout(rgba32f, binding = 0) uniform image2D currentFrameBuffer;
layout(rgba32f, binding = 1) uniform image2D hitBuffer;
layout(rgba32f, binding = 2) uniform image2D accumulationBuffer;

layout(std430, binding = 3) readonly buffer SpheresData    { Sphere spheres[]; };

layout(std430, binding = 4) readonly buffer VerticesData   { float vertices[]; };

layout(std430, binding = 5) readonly buffer TrianglesData  { Triangle triangles[]; };

layout(std430, binding = 6) readonly buffer MaterialsData  { Material materials[]; };

layout(std430, binding = 7) readonly buffer ObjectIDs      { PrimitiveInfo primitiveIDs[]; };

layout(std430, binding = 8) readonly buffer ObjectsBVH     { Node objectsBVH[]; };

layout(location = 1) uniform float uTime;
layout(location = 2) uniform int   uFrameCount;
layout(location = 3) uniform vec3  position;
layout(location = 4) uniform mat4  invView;
layout(location = 5) uniform mat4  invProj;
// layout(location = 6) uniform ivec4 scenePrimitives;

// -------------------------------------------------
//                   Random Utils 
// -------------------------------------------------

// Global seed for generating random numbers
uint gSeed = 0;

// Generates random number between [0,1)
float randomFloat()
{
    // PCG hash
    uint state = gSeed * 747796405U + 2891336453U;
    uint word = ((state >> ((state >> 28U) + 4U)) ^ state) * 277803737U;
    gSeed = (word >> 22U) ^ word;
    
    return gSeed/float(0xffffffffU);
}

// Generates random number between [min, max)
float randomFloatRange(in float min, in float max)
{
    return min + (max-min)*randomFloat();
}

// -------------------------------------------------
//                    Vec Utils 
// -------------------------------------------------

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
    r.energy = vec3(1.0);
    return r;
}

vec3 pointAt(in Ray r, in float t){return r.orig + t*r.dir;}

// -------------------------------------------------
//                 Interval Functions 
// -------------------------------------------------

Interval interval(in float min_, in float max_)
{
    Interval i;
    i.min = min_;
    i.max = max_;
    return i;
}

bool contains(in Interval i, in float x){return i.min <= x && x <= i.max;}

bool surrounds(in Interval i, in float x){return i.min < x && x < i.max;}

// -------------------------------------------------
//             Physical Based Rendering 
// -------------------------------------------------

float distributionGGX(in float NdotH, in float roughness)
{
    float a = roughness*roughness;
    float a2 = a*a;
    float NdotH2 = NdotH*NdotH;

    float num   = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return num / denom;
}

float geometrySchlickGGX(in float NdotV, in float roughness)
{
    float a = (roughness + 1.0);
    float k = (a*a) / 8.0;

    float num   = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return num / denom;
}
                                                                               
float geometrySmith(in float NdotV, in float NdotL, in float roughness)
{
    float ggx2 = geometrySchlickGGX(NdotV, roughness);
    float ggx1 = geometrySchlickGGX(NdotL, roughness);

    return ggx1 * ggx2;
}

vec3 fresnelSchlick(in float cosTheta, in vec3 F0)
{
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

vec3 cosineSample(vec3 N)
{
    float cosTheta = sqrt(randomFloat());
    float sinTheta = sqrt(1.0f - cosTheta * cosTheta);
    float phi = 2 * PI * randomFloat();
    vec3 d = vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);

    vec3 helper = abs(N.x) < 0.999 ? vec3(1, 0, 0) : vec3(0.0, 0.0, 1.0);
    vec3 tangent = normalize(cross(N, helper));
    vec3 bitangent = cross(N, tangent);
    d = normalize(tangent * d.x + bitangent * d.y + N * d.z);
    return d;
}

vec3 importanceSampleGGX(in vec3 N, in vec3 V, in float roughness)
{
    float a = roughness * roughness;
    float phi = 2 * PI * randomFloat();

    float xi = randomFloat(); 
    float cosTheta = sqrt((1.0 - xi) / (1.0 + (a * a - 1.0) * xi));
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

    vec3 m = vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
    vec3 up = abs(N.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
    vec3 tangent = normalize(cross(N, up));
    vec3 bitangent = cross(N, tangent);
    vec3 h = normalize(tangent * m.x + bitangent * m.y + N * m.z);
    return h;
}

float importanceSampleGGX_PDF(float NDF, float NdotH, float VdotH)
{
    return NDF * NdotH / (4 * VdotH);
}

// -------------------------------------------------
//                   Hit Functions
// -------------------------------------------------

bool isEmissive(in Material m)
{
    return (m.emission[0] > 0) || (m.emission[1] > 0) || (m.emission[2] > 0); 
}

void setFaceNormal(inout HitRecord rec, in Ray r, in vec3 outwardNormal)
{    
    rec.frontFace = dot(r.dir, outwardNormal) < 0;
    if (rec.frontFace) rec.normal = outwardNormal;
    else rec.normal = -outwardNormal;
}

bool hitSphere(in Sphere s, in Ray r, in Interval rayT, inout HitRecord rec)
{
    // Ray Intersection
    vec3 oc = r.orig - s.center.xyz;
    float a = dot(r.dir, r.dir); // GLSL has no squared length function, that is why dot product is used.
    float half_b = dot(oc, r.dir);
    float c = dot(oc, oc) - s.radius*s.radius;
    
    float disc = half_b*half_b - a*c;
    
    if (disc < 0) return false;
    float sqrtDisc = sqrt(disc);

    // find the nearest root that lies in the acceptable range
    float root = (-half_b - sqrtDisc)/a;
    if (!contains(rayT, root))
    {
        root = (-half_b + sqrtDisc)/a;
        if (!contains(rayT, root)) return false;
    }

    rec.t = root;
    rec.hitPoint = pointAt(r, rec.t);
    vec3 outwardNormal = (rec.hitPoint - s.center.xyz)/s.radius; // By dividing all componentes by the sphere radius the vector is normalized
    setFaceNormal(rec, r, outwardNormal);
    rec.material = materials[s.matIdx];   

    return true;
}

bool hitTriangle(in Triangle tri, in Ray r, in Interval rayT, inout HitRecord rec)
{
    vec3 a = vertex(tri.a);
    vec3 b = vertex(tri.b);
    vec3 c = vertex(tri.c);
    vec3 edge1 = b - a;
    vec3 edge2 = c - a;
    vec3 rayCrossEdge2 = cross(r.dir, edge2);
    float det = dot(edge1, rayCrossEdge2);

    if (abs(det) < EPSILON) return false;

    float invDet = 1.0/det;
    
    vec3 s = r.orig - a;
    float u = invDet * dot(s, rayCrossEdge2); 
    if (u < 0 || u > 1) return false;   

    vec3 sCrossEdge1 = cross(s, edge1);
    float v = invDet * dot(r.dir, sCrossEdge1);
    if (v < 0 || u + v > 1) return false;

    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = invDet * dot(edge2, sCrossEdge1);
    if (!surrounds(rayT, t)) return false;

    rec.t = t;
    rec.hitPoint = pointAt(r, rec.t);
    vec3 outwardNormal = normalize(cross(edge1,edge2));
    setFaceNormal(rec, r, outwardNormal);
    rec.material = materials[tri.matIdx];
    return true;
}

float hitBB1(in Ray r, in Node node, in float nearestHit) {
    vec3 invDir = 1 / r.dir;
    vec3 tMin = (node.minB.xyz - r.orig) * invDir;
    vec3 tMax = (node.maxB.xyz - r.orig) * invDir;
    vec3 t1 = min(tMin, tMax);
    vec3 t2 = max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar = min(min(t2.x, t2.y), t2.z);
    if (tNear <= tFar && tFar > 0 && tNear < nearestHit) {
        return tNear;
    }
    else {
        return 999999999;
    }
}

bool hitBB(in Ray r, in Node node, in float nearestHit, in vec3 invDir)
{
    vec3 tNear = (node.minB.xyz - r.orig) * invDir;
    vec3 tFar  = (node.maxB.xyz - r.orig) * invDir;
    vec3 tMin = min(tNear,tFar);
    vec3 tMax = max(tNear,tFar);
    float t0 = max(max(tMin.x, tMin.y), tMin.z);
    float t1 = min(min(tMax.x, tMax.y), tMax.z);
    return t0 <= t1 && t1 > 0 && t0 < nearestHit;
}

HitRecord rdsbTraversal(in Ray r, in Interval rayT)
{
    HitRecord rec;
    rec.scatterRay = r;
    rec.hit = false;
    float nearestHit = rayT.max;

    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;

    // vec3 invDir = vec3(1.0/r.dir.x,1.0/r.dir.y,1.0/r.dir.z);
    vec3 invDir = 1.0/r.dir;
    vec3 dirIsNeg = vec3(invDir.x < 0, invDir.y < 0, invDir.z > 0);

    while(true) {
        Node node = objectsBVH[currentNodeIndex];
        int nPrimitives = node.nPrimitives;
        int primitiveOffset = node.primitiveOffset;
        if (hitBB(r, node, nearestHit, invDir)){
            if (nPrimitives > 0){
                for(int i = 0; i < nPrimitives; i++) {
                    PrimitiveInfo objInfo = primitiveIDs[primitiveOffset + i];
                    
                    if (objInfo.type == SPHERE && hitSphere(spheres[objInfo.idx], r, interval(0.001, nearestHit), rec)){
                        rec.hit = true;
                        nearestHit = rec.t;
                    }
                    else if (hitTriangle(triangles[objInfo.idx], r, interval(0.001, nearestHit), rec)){
                        rec.hit = true;
                        nearestHit = rec.t;
                    }
                }
                if(toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            }
            else {
                if (dirIsNeg[node.splitAxis] > 0.0){
                    nodesToVisit[toVisitOffset++] = primitiveOffset;
                    currentNodeIndex = primitiveOffset + 1;
                } 
                else {
                    nodesToVisit[toVisitOffset++] = primitiveOffset + 1;
                    currentNodeIndex = primitiveOffset;
                }
            }
        }
        else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return rec;
}

HitRecord dbTraversal(in Ray r, in Interval rayT)
{
    HitRecord rec;
    rec.hit = false;
    rec.scatterRay = r;
    float nearestHit = rayT.max;
    
    Node node = objectsBVH[0];
    Node stack[30];
    int stackPos = 0;
    
    vec3 invDir = 1.0/r.dir;
    if(!hitBB(r, node, nearestHit, 1.0/r.dir)) return rec;

    while(true) {
        const int objectCount = node.nPrimitives;
        const int content = node.primitiveOffset;
        
        if (objectCount > 0) {
            
            for(int i = 0; i < objectCount; i++) {
                PrimitiveInfo objInfo = primitiveIDs[i + content];
                
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

            float dist1 = hitBB1(r, leftChild, nearestHit);
            float dist2 = hitBB1(r, rightChild, nearestHit);

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

bool scatter(inout HitRecord rec)
{
    if (isEmissive(rec.material)) return false;
    Material m = rec.material;

    float roulette = randomFloat();
    vec3 reflectionDir;
    vec3 V = normalize(-rec.scatterRay.dir);
    
    float diffuseRatio = 0.5 * (1.0 - m.metalness);
    float specularRatio = 1 - diffuseRatio;

    if (roulette < diffuseRatio)
        // sample diffuse
        reflectionDir =  cosineSample(rec.normal);
    else{
        // sample specular
        vec3 sampledH = importanceSampleGGX(rec.normal, V, m.roughness);
        reflectionDir = 2.0 * dot(V, sampledH) * sampledH - V;
        reflectionDir = normalize(reflectionDir);
    }
    
    vec3 L = normalize(reflectionDir);
    vec3 H = normalize(V + L);

    float NdotL = max(abs(dot(rec.normal, L)), 0.0);
    float NdotH = max(abs(dot(rec.normal, H)), 0.0);
    float NdotV = max(abs(dot(rec.normal, V)), 0.0);
    float VdotH = max(abs(dot(V, H)), 0.0);
    
    vec3 f0 = vec3(0.04);
    f0 = mix(f0, m.albedo.xyz, m.metalness);
    
    float NDF = distributionGGX(NdotH, m.roughness);
    float G   = geometrySmith(NdotV, NdotL, m.roughness);
    vec3  F   = fresnelSchlick(VdotH, f0);

    vec3 ks = F; 
    vec3 kd = vec3(1.0) - ks;
    kd *= 1.0 - m.metalness;

    vec3  diffuseBRDF = m.albedo.xyz / PI; 
    float diffusePDF  = NdotL / PI;

    vec3  numerator    = NDF * G * F;
    float denominator  = 4.0 * NdotV * NdotL + 0.0001;
    vec3  specularBRDF = numerator/denominator;
    float specularPDF  = importanceSampleGGX_PDF(NDF, NdotH, VdotH);

    vec3 totalBRDF = (diffuseBRDF * kd + specularBRDF) * NdotL;
    float totalPDF = diffuseRatio * diffusePDF + specularRatio * specularPDF;

    rec.scatterRay.orig = rec.hitPoint;
    rec.scatterRay.dir  = reflectionDir;
    if (totalPDF > 0.0) rec.scatterRay.energy *= totalBRDF / totalPDF; 
    return true;
}

vec3 rayColor(in Ray r)
{   
    vec3 indirectLighting = vec3(1.0);
    vec3 directLighting = vec3(0.0);
    vec3 contribution = vec3(0.0);
    vec3 skyColor = vec3(1.0);
    
    HitRecord rec;
    rec.scatterRay = r;
    
    for(int bounce = 0; bounce <= MAX_BOUNCES; bounce++)
    {
        rec = rdsbTraversal(rec.scatterRay, interval(0.001, 999999));
        
        if(!rec.hit){
            contribution += indirectLighting * skyColor + directLighting;
            break;
        }
        else if (bounce == MAX_BOUNCES) break; 

        // Indirect Illumination
        if (scatter(rec)) indirectLighting *= rec.scatterRay.energy;
        if (bounce == 0) imageStore(hitBuffer, pixelCoords, vec4(rec.hitPoint, 0));
    }
    return contribution;  
}

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

    const vec4 target = invProj * vec4(x, y, 1, 1);
    const vec3 direction = vec3(invView  * vec4(normalize(target.xyz), 0));
    
    Ray r = ray(position, direction);
    vec3 pixelColor = rayColor(r);
    
    // Gamma correction
    pixelColor = pixelColor / (pixelColor + vec3(1.0));
    pixelColor = pow(pixelColor, vec3(1.0/2.2));

    vec3 accumColor;
    if (uFrameCount > 1){        
        accumColor = imageLoad(accumulationBuffer, pixelCoords).rgb;
        accumColor += pixelColor;
        pixelColor = accumColor/uFrameCount;
    } 
    else accumColor = pixelColor;

    imageStore(currentFrameBuffer, pixelCoords, vec4(pixelColor, 1.0));
    imageStore(accumulationBuffer, pixelCoords, vec4(accumColor, 1.0));
}
