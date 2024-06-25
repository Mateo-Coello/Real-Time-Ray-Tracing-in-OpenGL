#ifndef HITTABLE_H
#define HITTABLE_H

#include <cglm/cglm.h>
#include <stdlib.h>
#include "tinyobj_loader_c.h"

#define LAMBERTIAN 0
#define METAL 1
#define DIELECTRIC 2
#define EMISSIVE 3

#define SPHERE 0
#define TRIANGLE 1
#define LIGHT 0

typedef struct
{
  vec4 albedo;
  vec4 specular;
  vec4 emission;
  float roughness;
  float metalness;
} Material;

typedef struct
{
  vec4 center;
  float radius;
  int matIdx;
} Sphere;

typedef struct
{
  int a; // first vertex index
  int b; // second vertex index
  int c; // third vertex index
  vec4 centroid;
  vec4 normal;
  int matIdx;
} Triangle;

typedef struct
{
  int idx;
  int type;
} PrimitiveInfo;

Material material(tinyobj_material_t mat);
Material lambertian(vec3 color);
Material metal(vec3 color, float fuzz);
Material dielectric(float refIdx);
Material emissive(vec3 color, float emissionPower);
Material* genMaterialBuffer(int count);
Sphere sphere(vec3 center, float radius, int material);
Sphere* genSphereBuffer(int count);
int* genSphereIDs(int count);
void triangleVertex(const float* vertices, int nVert, vec3 v);
void triangle(Triangle* t, vec3 v0, vec3 v1, vec3 v2, int matIdx);

#endif
