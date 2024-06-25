#version 460 core
layout(local_size_x = 8, local_size_y = 4, local_size_z = 1) in;
layout(rgba32f, binding = 0) uniform image2D screen;
layout(location = 0) uniform float timeSeed;

#define MAX_BOUNCES ()
#define MAX_FLOAT (1./0.)

uint pcgHash(in uint seed)
{
    uint state = seed * 747796405U + 2891336453U;
    uint word = ((state >> ((state >> 28U) + 4U)) ^ state) * 277803737U;
    return (word >> 22U) ^ word;
}

float randomFloat(inout uint seed)
{
    seed = pcgHash(seed);
    return seed/float(0xffffffffU);
}

vec3 randomVec3(inout uint seed)
{
    return vec3(
            randomFloat(seed) * 2 - 1,
            randomFloat(seed) * 2 - 1,
            randomFloat(seed) * 2 - 1);
}

vec3 randomVec3InUnitSphere(inout uint seed)
{
    vec3 v;
    while (true)
    {
        v = randomVec3(seed);
        if (length(v)<1){return v;}
    } 
}

vec3 randomUnitVec3(inout uint seed)
{
    return normalize(randomVec3InUnitSphere(seed));
}

struct Ray
{
    vec3 orig;
    vec3 dir;    
};    

Ray ray(vec3 origin, vec3 direction) 
{   
    Ray r;
    r.orig = origin;
    r.dir = direction;
    return r;
}

vec3 pointAt(Ray r, float t)
{
    return r.orig + t*r.dir;
}

struct HitRecord
{
    vec3 hitPoint;
    vec3 normal;
    float t;
    bool frontFace;        
};

void setFaceNormal(inout HitRecord rec, Ray r, vec3 outwardNormal)
{    
    rec.frontFace = dot(r.dir, outwardNormal) < 0;
    rec.normal = rec.frontFace ? outwardNormal : -outwardNormal;
}

struct Sphere
{
    vec3 center;
    float radius;        
};

Sphere sphere(vec3 center_, float radius_)
{
    Sphere s;
    s.center = center_;
    s.radius = radius_;
    return s;
}

struct Interval
{
    float min;
    float max;      
};

Interval interval(float min_, float max_)
{
    Interval i;
    i.min = min_;
    i.max = max_;
    return i;
}

bool contains(Interval i, float x){return i.min <= x && x <= i.max;}

bool surrounds(Interval i, float x){return i.min < x && x < i.max;}

bool hitSphere(Sphere s, Ray r, Interval rayT, inout HitRecord rec)
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

    return true;
}

bool hitSpheres(Sphere[2] world, Ray r, Interval rayT, inout HitRecord rec)
{
    HitRecord tempRec;
    bool hitAnything = false;
    float closest = rayT.max;
    for (int i=0; i<2; i++)
    {
        if (hitSphere(world[i], r, interval(rayT.min, closest), tempRec)){
            hitAnything = true;
            closest = tempRec.t;
            rec = tempRec;
        }
    }
    return hitAnything;
}

vec3 rayColor(Ray r, Sphere[2] world)
{
    HitRecord rec;

    int bounce = 0;
    while(bounce < MAX_BOUNCES)
    {
        if (hitSpheres(world, r, interval(0.01, MAX_FLOAT), rec)){
            vec3 dir = rec.normal + randomUnitVec3()
            return 0.5*(rec.normal + 1.0);
        }
    }
        
    vec3 unit_direction = normalize(r.dir);
    float a = 0.5*(unit_direction.y + 1.0);
    return (1.0-a)*vec3(1.0,1.0,1.0) + a*vec3(0.5, 0.7, 1.0);
}

struct Camera
{
    vec3 lookFrom, lookAt, vUp, pixel00Loc, pixelDeltaU, pixelDeltaV, u, v, w;
    float imageWidth, imageHeight;
};

Camera initCamera(float iWidth, float iHeight, float vfov, vec3 lookFrom_, vec3 lookAt_)
{
    Camera c;
    // Set initial parameters
    c.lookFrom = lookFrom_;
    c.lookAt = lookAt_;
    c.vUp = vec3(0.0, 1.0, 0.0);
    c.imageWidth = iWidth;
    c.imageHeight = iHeight; 

    // Determine the viewport dimensions
    float focalLength = length(c.lookFrom - c.lookAt);
    float h = tan(radians(vfov) / 2);
    float viewportHeight = 2 * h * focalLength;
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
    vec3 viewportUpperLeft = c.lookFrom - focalLength * c.w - viewportU/2 - viewportV/2;
    c.pixel00Loc = viewportUpperLeft + 0.5 * (c.pixelDeltaU + c.pixelDeltaV);

    return c;
}

// void main()
// {    
//     // Pixel Coords
//     ivec2 pixel_coords = ivec2(gl_GlobalInvocationID.xy);
//     ivec2 dims = imageSize(screen);

//     // Camera
//     vec3 lookFrom = vec3(0.0, 0.0, 0.0);
//     vec3 lookAt = vec3(0.0, 0.0, -1.0);
//     Camera cam = initCamera(dims.x, dims.y, 90., lookFrom, lookAt);
//     // float focalLength = 1.0;
//     // vec3 cam_c = vec3(0.0, 0.0, 0.0);
    
//     // float viewportHeight = 2.0;
//     // float viewportWidth = viewportHeight * dims.x/dims.y;
//     // vec3 viewportU = vec3(viewportWidth, 0, 0);
//     // vec3 viewportV = vec3(0, viewportHeight, 0);
//     // vec3 pixelDeltaU = viewportU/dims.x;
//     // vec3 pixelDeltaV = viewportV/dims.y;
//     // vec3 viewportUpperLeft = cam_c - vec3(0, 0, focalLength) - viewportU/2 - viewportV/2;
//     // vec3 pixel00Loc = viewportUpperLeft + 0.5 * (pixelDeltaU + pixelDeltaV);
        
//     vec3 pixel_center = cam.pixel00Loc + pixel_coords.x*cam.pixelDeltaU + pixel_coords.y*cam.pixelDeltaV;
//     vec3 ray_d = normalize(pixel_center - lookFrom);
//     Ray r = ray(lookFrom, ray_d);
   
//     //World
//     Sphere world[2] = {
//         {vec3(0, 0, -2), 0.5},
//         {vec3(0, -100.5,-2), 100}};
    
//     vec4 pixelColor = vec4(rayColor(r, world), 1.0);
     
//     imageStore(screen, pixel_coords, pixelColor);
// }

// void main()
// {
// 	vec4 pixel = vec4(0.075, 0.133, 0.173, 1.0);
// 	ivec2 pixel_coords = ivec2(gl_GlobalInvocationID.xy);
	
// 	ivec2 dims = imageSize(screen);
// 	float x = -(float(pixel_coords.x * 2 - dims.x) / dims.x); // transforms to [-1.0, 1.0]
// 	float y = -(float(pixel_coords.y * 2 - dims.y) / dims.y); // transforms to [-1.0, 1.0]

// 	float fov = 90.0;
// 	vec3 cam_o = vec3(0.0, 0.0, -tan(fov / 2.0));
// 	vec3 ray_o = vec3(x, y, 0.0);
// 	vec3 ray_d = normalize(ray_o - cam_o);

// 	vec3 sphere_c = vec3(0.0, 0.0, -5.0);
// 	float sphere_r = 1.0;

// 	vec3 o_c = ray_o - sphere_c;
// 	float b = dot(ray_d, o_c);
// 	float c = dot(o_c, o_c) - sphere_r * sphere_r;
// 	float intersectionState = b * b - c;
// 	vec3 intersection = ray_o + ray_d * (-b + sqrt(b * b - c));

// 	if (intersectionState >= 0.0)
// 	{
// 		pixel = vec4((normalize(intersection - sphere_c) + 1.0) / 2.0, 1.0);
// 	}

// 	imageStore(screen, pixel_coords, pixel);
// }