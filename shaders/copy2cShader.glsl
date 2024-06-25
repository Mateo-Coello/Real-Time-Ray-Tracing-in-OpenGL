#version 460 core
layout(local_size_x = 8, local_size_y = 4) in;
layout(rgba32f, binding = 0) uniform image2D screen;
layout(location = 0) uniform float uTime;
layout(location = 1) uniform vec3 position;
layout(location = 2) uniform vec3 front;
layout(location = 3) uniform vec3 right;
layout(location = 4) uniform vec3 up;
layout(location = 5) uniform vec3 vfov;

// -------------------------------------------------
//                  GLOBAL PARAMETERS 
// -------------------------------------------------

#define MAX_SPHERES 100
#define MAX_BOUNCES 5
#define SAMPLES_PER_PIXEL 10
#define INFINITY (1./0.)
#define NEG_INFINITY (-INFINITY)
#define EPSILON 0.0000001

// -------------------------------------------------
//                   Random Utils 
// -------------------------------------------------

// Global seed for generating random numbers
uint gSeed = 0;

// Hash function for generating random number
uint pcgHash(in uint seed)
{
    uint state = seed * 747796405U + 2891336453U;
    uint word = ((state >> ((state >> 28U) + 4U)) ^ state) * 277803737U;
    return (word >> 22U) ^ word;
}

// Generates random number between [0,1)
float randomFloat(inout uint seed)
{
    seed = pcgHash(seed);
    return seed/float(0xffffffffU);
}

// Generates random number between [min, max)
float randomFloatRange(inout uint seed, in float min, in float max)
{
    return min + (max-min)*randomFloat(seed);
}

// -------------------------------------------------
//                    Vec3 Utils 
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

// -------------------------------------------------
//                       Rays 
// -------------------------------------------------

struct Ray
{
    vec3 orig;
    vec3 dir;    
};    

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
//                     Interval
// -------------------------------------------------

struct Interval
{
    float min;
    float max;      
};

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
//                     Materials
// -------------------------------------------------

#define LAMBERTIAN 0
#define METAL 1
#define DIELECTRIC 2

struct Material
{
    int type;
    vec3 albedo;
    float fuzz;
    float refIdx;            
};

Material lambertian(vec3 albedo)
{
    Material m;
    m.type = LAMBERTIAN;
    m.albedo = albedo;
    return m;
}

Material metal(in vec3 albedo, in float fuzz)
{
    Material m;
    m.type = METAL;
    m.albedo = albedo;
    m.fuzz = fuzz;
    return m;
}

Material dielectric(in float refIdx)
{
    Material m;
    m.type = DIELECTRIC;
    m.refIdx = refIdx;
    return m;
}

// -------------------------------------------------
//                       AABBs
// -------------------------------------------------
struct AABB
{
    Interval x, y, z;       
};

AABB aabb(in Interval x, in Interval y, in Interval z)
{
    AABB bb;
    bb.x = x;
    bb.y = y;
    bb.z = z;
    return bb;
}

// Treat the two points a and b as extrema for the bounding box, so we don't require a
// particular minimum/maximum coordinate order.
AABB aabb(in vec3 a, in vec3 b)
{
    AABB bb;
    bb.x = interval(min(a.x, b.x), max(a.x, b.x));
    bb.y = interval(min(a.y, b.y), max(a.y, b.y));
    bb.z = interval(min(a.z, b.z), max(a.z, b.z));
    return bb;
}

Interval axis(in AABB bb, in int n)
{
    if (n==1) return bb.y;
    if (n==2) return bb.z;
    return bb.x;
}

bool hit(in AABB bb, in Ray r, inout Interval rayT)
{
    bool noHit = false;
    for (int i = 0; i < 0; i++)
    {
        float invD = 1/r.dir[i];
        float orig = r.orig[i];

        float t0 = (axis(bb, i).min - orig) * invD;
        float t1 = (axis(bb, i).max - orig) * invD;

        if (invD < 0)
        {
            float temp;
            temp = t0;
            t0 = t1;
            t1 = temp;
        }

        if (t0 > rayT.min) rayT.min = t0;
        if (t1 > rayT.max) rayT.max = t1;

        if (rayT.max <= rayT.min) 
        {
            noHit = true;
            break;
        }
    }
    return noHit ? false : true;
}

// -------------------------------------------------
//                    Hit Record
// -------------------------------------------------

struct HitRecord
{
    vec3 hitPoint;
    vec3 normal;
    float t;
    bool frontFace;
    Material material;       
};

void setFaceNormal(inout HitRecord rec, in Ray r, in vec3 outwardNormal)
{    
    rec.frontFace = dot(r.dir, outwardNormal) < 0;
    if (rec.frontFace)
    {
        rec.normal = outwardNormal;
    }
    else {rec.normal = -outwardNormal;}
    // rec.normal = rec.frontFace ? outwardNormal : -1*outwardNormal;
}

// -------------------------------------------------
//                     Spheres
// -------------------------------------------------

struct Sphere
{
    vec3 center;
    float radius;
    Material material;
    AABB bb;      
};

Sphere sphere(in vec3 center, in float radius, in Material material)
{
    Sphere s;
    s.center = center;
    s.radius = radius;
    s.material = material;
    // Generate bounding box for sphere
    vec3 rvec = vec3(radius);
    s.bb = aabb(center - rvec, center + rvec);
    return s;
}

// -------------------------------------------------
//                     Hittable
// -------------------------------------------------
#define SPHERE 0
#define BOX 1
// #define  2

struct Hittable
{
    int type;
    Sphere s;
    AABB bb;
};

// -------------------------------------------------
//             Bounding Volume Hierarchy
// -------------------------------------------------

// #define STACK_SIZE 32

// struct BvhNode
// {
//     Sphere left;
//     Sphere right;
//     AABB bb;        
// };

// bool hit(in BvhNode node, in Ray r, in Interval rayT, inout HitRecord rec)
// {
//     if (hit(node.bb, r, rayT)) return false;

//     bool hitLeft = hitSphere(node.left, ray, rayT, rec)
//     bool hitLeft = hitSphere(node.right, ray, rayT, rec)

//     return hitLeft || hitRight;
// }

// bool traverseBVH(vec3)

// -------------------------------------------------
//                      Scene
// -------------------------------------------------

struct Scene
{
    int numSpheres;
    Sphere spheres[MAX_SPHERES];       
};

Scene generateSimpleSpheresScene()
{
    Scene s;
    s.numSpheres = 4;
    
    s.spheres[0] = sphere(vec3(-1, 0, -1), 0.5, dielectric(1.5));                    // Left
    s.spheres[1] = sphere(vec3( 1, 0, -1), 0.5, metal(vec3(0.3, 1.0, 0.2), 0.0));    // Right
    s.spheres[2] = sphere(vec3( 0, 0, -1), 0.5, lambertian(vec3(0.7, 0.3, 0.3)));    // Center
    s.spheres[3] = sphere(vec3(0, -100.5,-1), 100, lambertian(vec3(0.8, 0.8, 0.0))); // Ground

    return s;
}

Scene generateComplexSpheresScene()
{
    int numSpheres = 0;
    Scene s;
    
    s.spheres[numSpheres++] = sphere(vec3(0, -1000, 0), 1000, lambertian(vec3(0.5))); // Ground 
    s.spheres[numSpheres++] = sphere(vec3(4, 1, 0), 1.0, metal(vec3(0.7, 0.6, 0.5), 0.0)); // Big Metal Sphere
    s.spheres[numSpheres++] = sphere(vec3(0, 1, 0), 1.0, dielectric(1.5));                 // Big Glass Sphere
    s.spheres[numSpheres++] = sphere(vec3(-4, 1, 0), 1.0, metal(vec3(0.34, 0.55, 0.49), 0.1)); // Big Metal Sphere
    
    s.spheres[numSpheres++] = sphere(vec3(-2.8, 0.2, 2.7), 0.2, lambertian(vec3(0.7, 0.4, 1.0))); 
    s.spheres[numSpheres++] = sphere(vec3(-2.5, 0.2, 1.4), 0.2, lambertian(vec3(0.0, 0.8, 0.6))); 
    s.spheres[numSpheres++] = sphere(vec3(-2.3, 0.2, 3.6), 0.2, lambertian(vec3(1.0, 0.6, 0.0))); 
    s.spheres[numSpheres++] = sphere(vec3(-2, 0.2, 4.3), 0.2, lambertian(vec3(0.64, 0.64, 0.46))); 
    s.spheres[numSpheres++] = sphere(vec3(-2.8, 0.2, -2.7), 0.2, lambertian(vec3(0.8, 0.2, 0.0))); 
    s.spheres[numSpheres++] = sphere(vec3(-2.5, 0.2, -1.4), 0.2, lambertian(vec3(0.0, 0.4, 0.6))); 
    s.spheres[numSpheres++] = sphere(vec3(-2.3, 0.2, -3.6), 0.2, lambertian(vec3(1.0, 0.4, 0.4))); 
    s.spheres[numSpheres++] = sphere(vec3(-2, 0.2, -4.3), 0.2, lambertian(vec3(1.0, 0.0, 0.0))); 

    s.spheres[numSpheres++] = sphere(vec3(-1, 0.2, 1.9), 0.2, lambertian(vec3(0.8, 0.4, 0.6))); 
    s.spheres[numSpheres++] = sphere(vec3(-1.7, 0.2, 4.3), 0.2, lambertian(vec3(0.6, 0.0, 1.0))); 
    s.spheres[numSpheres++] = spjamaicahere(vec3(-1.6, 0.2, 2.5), 0.2, lambertian(vec3(1.0, 0.6, 0.6))); 
    s.spheres[numSpheres++] = sphere(vec3(-1.3, 0.2, 3.1), 0.2, lambertian(vec3(0.6, 1.0, 0.2))); 
    s.spheres[numSpheres++] = sphere(vec3(-1.4, 0.2, 3.5), 0.2, lambertian(vec3(0.0, 0.8, 1.0))); 
    s.spheres[numSpheres++] = sphere(vec3(-1, 0.2, -1.9), 0.2, lambertian(vec3(0.4, 0.0, 0.4))); 
    s.spheres[numSpheres++] = sphere(vec3(-1.7, 0.2, -4.3), 0.2, lambertian(vec3(0.0, 0.6, 0.6))); 
    s.spheres[numSpheres++] = sphere(vec3(-1.6, 0.2, -2.5), 0.2, lambertian(vec3(1.0, 0.6, 0.0))); 
    s.spheres[numSpheres++] = sphere(vec3(-1.3, 0.2, -3.1), 0.2, lambertian(vec3(0.74, 0.34, 0.20))); 
    s.spheres[numSpheres++] = sphere(vec3(-1.4, 0.2, -3.5), 0.2, lambertian(vec3(0.38, 0.15, 0.41))); 
    
    s.spheres[numSpheres++] = sphere(vec3(2, 0.2, -2.4), 0.2, lambertian(vec3(1.0, 0.8, 0.36))); 
    s.spheres[numSpheres++] = sphere(vec3(2.2, 0.2, -4.3), 0.2, lambertian(vec3(0.35, 0.55, 0.49))); 
    s.spheres[numSpheres++] = sphere(vec3(2.5, 0.2, -1.6), 0.2, lambertian(vec3(0.88, 0.39, 0.46))); 
    s.spheres[numSpheres++] = sphere(vec3(2.7, 0.2, -3.0), 0.2, lambertian(vec3(0.40, 0.30, 0.24))); 
    s.spheres[numSpheres++] = sphere(vec3(2.9, 0.2, -3.7), 0.2, lambertian(vec3(1.0, 0.94, 0.59))); 
    s.spheres[numSpheres++] = sphere(vec3(2, 0.2, 2.4), 0.2, lambertian(vec3(0.51, 0.72, 0.29))); 
    s.spheres[numSpheres++] = sphere(vec3(2.2, 0.2, 4.3), 0.2, lambertian(vec3(1.0, 0.7, 0.21))); 
    s.spheres[numSpheres++] = sphere(vec3(2.5, 0.2, 1.6), 0.2, lambertian(vec3(0.24, 0.26, 0.26))); 
    s.spheres[numSpheres++] = sphere(vec3(2.7, 0.2, 3.0), 0.2, lambertian(vec3(0.76, 0.83, 0.86))); 
    s.spheres[numSpheres++] = sphere(vec3(2.9, 0.2, 3.7), 0.2, lambertian(vec3(0.85, 0.39, 0.35))); 
    
    s.spheres[numSpheres++] = sphere(vec3(1, 0.2, 2.4), 0.2, lambertian(vec3(1.0, 0.8, 0.36))); 
    s.spheres[numSpheres++] = sphere(vec3(1.2, 0.2, 4.3), 0.2, lambertian(vec3(0.35, 0.55, 0.49))); 
    s.spheres[numSpheres++] = sphere(vec3(1.5, 0.2, 1.6), 0.2, lambertian(vec3(0.88, 0.39, 0.46))); 
    s.spheres[numSpheres++] = sphere(vec3(1.7, 0.2, 3.0), 0.2, lambertian(vec3(0.40, 0.30, 0.24))); 
    s.spheres[numSpheres++] = sphere(vec3(1.9, 0.2, 3.7), 0.2, lambertian(vec3(1.0, 0.94, 0.59))); 
    s.spheres[numSpheres++] = sphere(vec3(1, 0.2, -2.4), 0.2, lambertian(vec3(0.51, 0.72, 0.29))); 
    s.spheres[numSpheres++] = sphere(vec3(1.2, 0.2, -4.3), 0.2, lambertian(vec3(1.0, 0.7, 0.21))); 
    s.spheres[numSpheres++] = sphere(vec3(1.5, 0.2, -1.6), 0.2, lambertian(vec3(0.24, 0.26, 0.26))); 
    s.spheres[numSpheres++] = sphere(vec3(1.7, 0.2, -3.0), 0.2, lambertian(vec3(0.76, 0.83, 0.86))); 
    s.spheres[numSpheres++] = sphere(vec3(1.9, 0.2, -3.7), 0.2, lambertian(vec3(0.85, 0.39, 0.35)));

    s.spheres[numSpheres++] = sphere(vec3(0.8, 0.2, -2.7), 0.2, lambertian(vec3(0.7, 0.4, 1.0))); 
    s.spheres[numSpheres++] = sphere(vec3(0.5, 0.2, -1.4), 0.2, lambertian(vec3(0.0, 0.8, 0.6))); 
    s.spheres[numSpheres++] = sphere(vec3(0.3, 0.2, -3.6), 0.2, lambertian(vec3(1.0, 0.6, 0.0))); 
    s.spheres[numSpheres++] = sphere(vec3(0, 0.2, -4.3), 0.2, lambertian(vec3(0.64, 0.64, 0.46))); 
    s.spheres[numSpheres++] = sphere(vec3(0, 0.2, -5), 0.2, lambertian(vec3(0.98, 0.84, 0.9))); 
    s.spheres[numSpheres++] = sphere(vec3(0.8, 0.2, 2.7), 0.2, lambertian(vec3(0.8, 0.2, 0.0))); 
    s.spheres[numSpheres++] = sphere(vec3(0.5, 0.2, 1.4), 0.2, lambertian(vec3(0.0, 0.4, 0.6))); 
    s.spheres[numSpheres++] = sphere(vec3(0.3, 0.2, 3.6), 0.2, lambertian(vec3(1.0, 0.4, 0.4))); 
    s.spheres[numSpheres++] = sphere(vec3(0, 0.2, 4.3), 0.2, lambertian(vec3(1.0, 0.0, 0.0))); 
    s.spheres[numSpheres++] = sphere(vec3(0, 0.2, 5), 0.2, lambertian(vec3(0.59, 0.81, 0.71))); 

    s.numSpheres = numSpheres;
    return s;
}

// -------------------------------------------------
//                 Sphere Hit Functions
// -------------------------------------------------

bool hitSphere(in Sphere s, in Ray r, in Interval rayT, inout HitRecord rec)
{
    // Ray Intersection
    vec3 oc = r.orig - s.center;
    float a = dot(r.dir, r.dir); // GLSL has no squared length function, that is why dot product is used.
    float half_b = dot(oc, r.dir);
    float c = dot(oc, oc) - s.radius*s.radius;
    
    float disc = half_b*half_b - a*c;
    
    if (disc < 0){return false;}
    float sqrtDisc = sqrt(disc);

    // find the nearest root that lies in the acceptable range
    float root = (-half_b - sqrtDisc)/a;
    if (root <= rayT.min || rayT.max <= root)
    {
        root = (-half_b + sqrtDisc)/a;
        if (root <= rayT.min || rayT.max <= root){return false;}
    }

    rec.t = root;
    rec.hitPoint = pointAt(r, rec.t);
    vec3 outwardNormal = (rec.hitPoint - s.center)/s.radius; // By dividing all componentes by the sphere radius the vector is normalized
    setFaceNormal(rec, r, outwardNormal);
    rec.material = s.material;   

    return true;
}

bool hitSpheres(in Scene scene, in Ray r, in Interval rayT, inout HitRecord rec)
{
    HitRecord tempRec;
    bool hitAnything = false;
    float closest = rayT.max;
    for (int i=0; i<scene.numSpheres; i++)
    {
        if (hitSphere(scene.spheres[i], r, interval(rayT.min, closest), tempRec)){
            hitAnything = true;
            closest = tempRec.t;
            rec = tempRec;
        }
    }
    return hitAnything;
}
// -------------------------------------------------
//                Material Scatter Ray
// -------------------------------------------------

// Computes the scatter ray depending on the material registered in the hit record
bool scatter(inout Ray r, in HitRecord rec, inout vec3 attenuation)
{
    if (rec.material.type == LAMBERTIAN)
    {
        vec3 scatterDir = rec.normal + randomUnitVec3(gSeed);
        scatterDir = nearZero(scatterDir) ? rec.normal : scatterDir;
        r = ray(rec.hitPoint, scatterDir);
        attenuation = rec.material.albedo;
        return true;
    }
    if (rec.material.type == METAL)
    {
        vec3 reflected = reflect(normalize(r.dir), rec.normal);
        r = ray(rec.hitPoint, reflected + rec.material.fuzz * randomUnitVec3(gSeed));
        attenuation = rec.material.albedo;
        return dot(r.dir, rec.normal) > 0;
    }
    if (rec.material.type == DIELECTRIC)
    {
        attenuation = vec3(1.0);
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
}

// -------------------------------------------------
//                   Ray Color
// -------------------------------------------------

vec3 rayColor(in Ray r, in Scene scene)
{
    HitRecord rec;

    vec3 color = vec3(1.0);
    vec3 attenuation = vec3(0.0);
    int bounces = 0;
    Ray scatterRay = r;
        
    while(bounces < MAX_BOUNCES)
    {   
        if (hitSpheres(scene, scatterRay, interval(0.001, INFINITY), rec)){
            if (scatter(scatterRay, rec, attenuation)){
                color *= attenuation;
            } 
            else {
                color = vec3 (0.0);
                break;
            }
        }
        else {
            vec3 unit_direction = normalize(r.dir);
            float a = 0.5*(unit_direction.y + 1.0);
            color *= (1.0-a)*vec3(1.0) + a*vec3(0.5, 0.7, 1.0);
            break;
        }
        bounces++;
    }
    
    if (bounces < MAX_BOUNCES){return color;}
    else {return vec3(0.0);}
}

// -------------------------------------------------
//                     Camera
// -------------------------------------------------

struct Camera
{
    vec3 lookFrom, lookAt, vUp, pixel00Loc, pixelDeltaU, pixelDeltaV, u, v, w, defocusDiskU, defocusDiskV;
    float imageWidth, imageHeight, defocusAngle, focusDist;
};

struct CamInitParams
{
    float iWidth, iHeight, vfov, defocusAngle, focusDist;
    vec3 lookFrom, lookAt;       
};

Camera initCamera(in CamInitParams params)
{
    Camera c;
    // Set initial parameters
    c.lookFrom = params.lookFrom; // lookFrom is the same as the center of the camera
    c.lookAt = params.lookAt;
    c.vUp = vec3(0.0, 1.0, 0.0);
    c.imageWidth = params.iWidth;
    c.imageHeight = params.iHeight;
    c.defocusAngle = params.defocusAngle;
    c.focusDist = params.focusDist;

    // Determine the viewport dimensions
    // float focalLength = length(c.lookFrom - c.lookAt);
    float h = tan(radians(params.vfov) / 2);
    float viewportHeight = 2 * h * c.focusDist;
    float viewportWidth = viewportHeight * c.imageWidth/c.imageHeight;

    // Calculate the u, v, w unit basis vectors for the camera coordinate frame
    c.w = normalize(c.lookFrom - c.lookAt);
    c.u = normalize(cross(c.vUp, c.w));
    c.v = cross(c.w, c.u);

    // Calculate the vectors across the horizontal and down the vertical viewport edges.
    vec3 viewportU = viewportWidth * c.u; // vector across viewport horizantal edge
    vec3 viewportV = viewportHeight * c.v; // vector down viewport vertical edge

    // Calculate the horizontal and vertical delta vectors from pixel to pixel.
    c.pixelDeltaU = viewportU/c.imageWidth;
    c.pixelDeltaV = viewportV/c.imageHeight;

    // Calculate the location of the upper left pixel.
    vec3 viewportUpperLeft = c.lookFrom - c.focusDist * c.w - viewportU/2 - viewportV/2;
    c.pixel00Loc = viewportUpperLeft + 0.5 * (c.pixelDeltaU + c.pixelDeltaV);

    // Calculate the camera defocus disk basis vectors
    float defocusRadius = c.focusDist * tan(radians(c.defocusAngle/2));
    c.defocusDiskU = c.u * defocusRadius;
    c.defocusDiskV = c.v * defocusRadius;
    
    return c;
}

// -------------------------------------------------
//                     Sampling
// -------------------------------------------------

// Returns a random point in the square surrounding a pixel at the origin 
vec3 pixelSampleSquare(in Camera c)
{
    float px = -0.5 + randomFloat(gSeed);
    float py = -0.5 + randomFloat(gSeed);
    return px * c.pixelDeltaU + py * c.pixelDeltaV;
}

// Get a randomly sampled camera ray for the pixel at location i,j
Ray getRay(in Camera c, in int i, in int j)
{
    vec3 pixelCenter = c.pixel00Loc + (i * c.pixelDeltaU) + (j * c.pixelDeltaV);
    vec3 pixelSample = pixelCenter + pixelSampleSquare(c);

    vec3 rayDirection = pixelSample - c.lookFrom;
    return ray(c.lookFrom, rayDirection);
}

// -------------------------------------------------
//                   Main Function
// -------------------------------------------------

void main()
{    
    // Pixel Coords
    ivec2 pixelCoords = ivec2(gl_GlobalInvocationID.xy);
    ivec2 dims = imageSize(screen);

    gSeed = floatBitsToUint(pixelCoords.x * pixelCoords.y * uTime);

    // Camera
    // vec3 lookFrom = vec3(12, 3, 1);
    // vec3 lookAt = vec3(0, 0, -1);
    // CamInitParams params = {dims.x, dims.y, 40., 0, 10, lookFrom, lookAt};
    // Camera cam = initCamera(params);

    float x = float(pixelCoords.x * 2 - dims.x)/dims.x;
    float y = float(pixelCoords.y * 2 - dims.y)/dims.y;

    // x = right.x/dims.x*(dims.x/dims.y)*2 * x;
    // y = up.y*2/dims.y * y;

    vec3 direction = front + x*((right*2*dims.x/dims.y)/dims.x) + y*((up*2)/dims.y);
    
    Scene scene = generateComplexSpheresScene();
    
    // Generating pixel color
    vec3 pixelColor = vec3(0.0);
    // for(int i = 0; i < SAMPLES_PER_PIXEL; i++)
    // {
    //     Ray r = getRay(cam, pixelCoords.x, pixelCoords.y);
    //     pixelColor += rayColor(r, scene);
    // }
    Ray r = ray(position,direction);
    pixelColor += rayColor(r, scene);
    // pixelColor /= SAMPLES_PER_PIXEL;
    pixelColor = sqrt(pixelColor);
    
    imageStore(screen, pixelCoords, vec4(pixelColor,1.0));
}
