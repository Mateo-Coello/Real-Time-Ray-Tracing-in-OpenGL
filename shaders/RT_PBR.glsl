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
    int count;
    int content;
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

struct SceneInfo
{
    int nSpheres;
    int nTri;
    int nObjects;
    int nLights;
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
layout(location = 4) uniform mat4  invView;
layout(location = 5) uniform mat4  invProj;
layout(location = 6) uniform ivec4 sceneInfo;

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

// Generates random vector whose components are between [0,1)
vec3 randomVec3()
{
    return vec3(
            randomFloat(),
            randomFloat(),
            randomFloat());
}

// Generates random vector whose components are between [min,max)
vec3 randomVec3Range(in float min, in float max)
{
    return vec3(
            randomFloatRange(min, max),
            randomFloatRange(min, max),
            randomFloatRange(min, max));
}

vec3 randomVec3InUnitDisk()
{
    vec3 v;
    while (true)
    {
        v = vec3(randomFloatRange(-1, 1), randomFloatRange(-1, 1), 0);
        if (dot(v,v) < 1){return v;}
    }
}

vec3 randomVec3InUnitSphere()
{
    vec3 v;
    while (true)
    {
        v = randomVec3Range(-1, 1);
        if (dot(v,v) < 1){return v;}
    } 
}

vec3 randomUnitVec3()
{
    return normalize(randomVec3InUnitSphere());
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
    r.energy = vec3(1.0);
    return r;
}

vec3 pointAt(in Ray r, in float t)
{
    return r.orig + t*r.dir;
}

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

float distributionGGX(vec3 N, vec3 H, float roughness)
{
    float a = roughness*roughness;
    float a2 = a*a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH*NdotH;

    float num   = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return num / denom;
}

float geometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r*r) / 8.0;

    float num   = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return num / denom;
}
                                                                               
float geometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = geometrySchlickGGX(NdotV, roughness);
    float ggx1 = geometrySchlickGGX(NdotL, roughness);

    return ggx1 * ggx2;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

float importanceSampleGGX_PDF(float NDF, float NdotH, float VdotH)
{
    return NDF * NdotH / (4 * VdotH);
}

// -------------------------------------------------
//                   Hit Functions
// -------------------------------------------------

bool isEmissive(Material m)
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

float hitBB(in Ray ray, in Node node, in float nearestHit) {

    vec3 tMin = (node.minB.xyz - ray.orig) / ray.dir;
    vec3 tMax = (node.maxB.xyz - ray.orig) / ray.dir;
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

HitRecord traceRay(in Ray r, in Interval rayT)
{
    HitRecord rec;
    rec.hit = false;
    rec.scatterRay = r;
    float nearestHit = rayT.max;
    
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

bool scatter1(inout HitRecord rec)
{
    if (isEmissive(rec.material)) return false;

    HitRecord shadowRec;
    
    vec3 N = rec.normal;
    vec3 V = normalize(-rec.scatterRay.dir); // vector pointing towards the view position
    vec3 f0 = vec3(0.04);
    f0 = mix(f0, rec.material.albedo.xyz, rec.material.metalness);

    // reflectance equation
    rec.irradiance = vec3(0.0);
    for (int i = sceneInfo[0] - sceneInfo[3]; i < sceneInfo[0]; i++)
    {    
        Sphere light = spheres[i];
        Material lightProp = materials[light.matIdx]; 
        vec3 L = light.center.xyz - rec.hitPoint; // vector pointing towards the light
        // shadowRec = traceRay(ray(rec.hitPoint, normalize(L)));
        
        float distance    = length(L);
        float attenuation = 1.0/(distance*distance);
        vec3  radiance    = lightProp.emission.xyz * attenuation;

        L = normalize(L);
        vec3 H = normalize(V + L); // half-way vector between the view vector and the light vector

        // cook-torrance brdf
        float NDF = distributionGGX(N, H, rec.material.roughness);
        float G   = geometrySmith(N, V, L, rec.material.roughness);
        vec3  F   = fresnelSchlick(max(dot(H, V), 0.0), f0);
        vec3 ks = F; 
        vec3 kd = vec3(1.0) - ks;
        kd *= 1.0 - rec.material.metalness;
        
        vec3 numerator    = NDF * G * F;
        float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001;
        vec3 specular     = numerator/denominator;

        // Add to outgoing radiance Lo
        float NdotL = max(dot(N, L), 0.0);
        rec.irradiance += (kd * rec.material.albedo.xyz / PI + specular) * radiance * NdotL;
    }
    vec3 scatterDir = N + randomUnitVec3();
    scatterDir = nearZero(scatterDir) ? N : scatterDir;
    rec.scatterRay = ray(rec.hitPoint, scatterDir);
    return true;
}

bool scatter(inout HitRecord rec)
{
    if (isEmissive(rec.material)) return false;
    
    float diffuseRatio = 0.5 * (1.0 - rec.material.metalness);
    float specularRatio = 1 - diffuseRatio;
   
    vec3 V = normalize(-rec.scatterRay.dir); 
    vec3 reflectionDir = normalize(rec.normal + randomUnitVec3());
    vec3 L = nearZero(reflectionDir) ? rec.normal : reflectionDir;    
    vec3 H = normalize(V + L);

    float NdotL = max(dot(rec.normal, L), 0.0);
    float NdotH = max(dot(rec.normal, H), 0.0);
    float NdotV = max(dot(rec.normal, V), 0.0);
    float VdotH = max(dot(V, H), 0.0);
    
    vec3 f0 = vec3(0.04);
    f0 = mix(f0, rec.material.albedo.xyz, rec.material.metalness);
    
    float NDF = distributionGGX(rec.normal, H, rec.material.roughness);
    float G   = geometrySmith(rec.normal, V, L, rec.material.roughness);
    vec3  F   = fresnelSchlick(max(dot(H, V), 0.0), f0);

    vec3 ks = F; 
    vec3 kd = vec3(1.0) - ks;
    kd *= 1.0 - rec.material.metalness;

    vec3  numerator    = NDF * G * F;
    float denominator  = 4.0 * NdotV * NdotL + 0.0001;
    vec3  specularBRDF = numerator/denominator;
    float specularPDF  = importanceSampleGGX_PDF(NDF, NdotH, VdotH);

    vec3  diffuseBRDF = rec.material.albedo.xyz / PI; 
    float diffusePDF  = NdotL / PI;

    vec3 totalBRDF = (diffuseBRDF * kd + specularBRDF) * NdotL;
    // vec3 totalBRDF = (diffuseBRDF * kd) * NdotL;
    float totalPDF = diffuseRatio * diffusePDF + specularRatio * specularPDF;
    // float totalPDF = diffusePDF;

    rec.scatterRay.orig = rec.hitPoint;
    rec.scatterRay.dir  = reflectionDir;
    if (totalPDF > 0.0)
    {
        rec.scatterRay.energy *= totalBRDF / totalPDF; 
    }
    return true;
}

vec3 rayColor(in Ray r)
{   
    vec3 light = vec3(0.0);
    vec3 indirectLighting = vec3(1.0);
    vec3 directLighting = vec3(1.0);
    vec3 contribution = vec3(1.0);
    vec3 skyColor = vec3(0.2, 0.2, 0.2);
    
    HitRecord rec;
    rec.scatterRay = r;
    
    for(int bounce = 0; bounce <= MAX_BOUNCES; bounce++)
    {
        rec = traceRay(rec.scatterRay, interval(0.001, 999999));
        
        if(!rec.hit){
            light = skyColor; 
            break;
        }

        // Indirect Illumination
        if (scatter(rec))
            indirectLighting *= rec.scatterRay.energy;
        else { 
            indirectLighting *= rec.material.emission.xyz;
            break;
        }
        
        // Direct Illumination
        if (bounce == 0)
        {
            Sphere light = spheres[1];
            Material lightProp = materials[light.matIdx];
            vec3 L = light.center.xyz - rec.hitPoint;
            float distance = length(L);
            L = L/distance;
            float attenuation = 1.0 / (distance * distance);
            vec3 radiance = lightProp.emission.xyz * attenuation;
    
            HitRecord shadowRec = traceRay(ray(rec.hitPoint, L), interval(0.001, distance));
            if (isEmissive(shadowRec.material))
                directLighting *= radiance * max(dot(rec.normal, L), 0.0) * rec.material.albedo.xyz;
            else directLighting = vec3(0.0);

            imageStore(hitBuffer, pixelCoords, vec4(rec.hitPoint, 0));
        }
        contribution = indirectLighting + directLighting;           
        // contribution = directLighting;           
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
