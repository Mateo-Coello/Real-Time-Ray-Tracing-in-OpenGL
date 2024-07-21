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
layout(location = 6) uniform vec3  sphereCenter;
layout(location = 7) uniform float sphereRadius;

struct Sphere
{
    vec3 center;
    float radius;
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
    vec3 hitPoint;
    vec3 normal;
    Interval rayT;
    float t;
    bool hit;
    bool frontFace;
};

Sphere sphere(in vec3 center, in float radius)
{
    Sphere s;
    s.center = center;
    s.radius = radius;
    return s;
}

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

void setFaceNormal(inout HitRecord rec, in Ray r, in vec3 outwardNormal)
{    
    rec.frontFace = dot(r.dir, outwardNormal) < 0;
    if (rec.frontFace) rec.normal = outwardNormal;
    else rec.normal = -outwardNormal;
}

bool hitSphere(in Ray r, in Sphere s, inout HitRecord rec)
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
    if (!contains(rec.rayT, root))
    {
        root = (-half_b + sqrtDisc)/a;
        if (!contains(rec.rayT, root)) return false;
    }

    rec.t = root;
    rec.hitPoint = pointAt(r, rec.t);
    vec3 outwardNormal = (rec.hitPoint - s.center.xyz)/s.radius; // By dividing all componentes by the sphere radius the vector is normalized
    setFaceNormal(rec, r, outwardNormal);

    return true;
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
    Sphere s = sphere(sphereCenter, sphereRadius);
    HitRecord rec;
    rec.rayT = interval(0, 999999);
    vec3 pixelColor = vec3(0.0);
    if (hitSphere(r, s, rec)) 
        pixelColor = (normalize(rec.hitPoint - s.center) + 1.0)/2.0;
    
    // Gamma correction
    pixelColor = pixelColor / (pixelColor + vec3(1.0));
    pixelColor = pow(pixelColor, vec3(1.0/2.2));

    imageStore(currentFrameBuffer, pixelCoords, vec4(pixelColor, 1.0));
}
