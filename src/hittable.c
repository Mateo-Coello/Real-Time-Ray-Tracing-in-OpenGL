#include "hittable.h"
#include "cglm/vec4.h"

Material material(tinyobj_material_t mat)
{
  Material m = {};
  glm_vec4(mat.diffuse, 0.0, m.albedo);
  glm_vec4(mat.specular, 0.0, m.specular);
  if (mat.emission[0] != 0) glm_vec4(mat.emission, 0.0, m.emission);
  m.roughness = 1 - mat.shininess;
  m.metalness = mat.dissolve;
  return m;
}

Sphere sphere(vec3 center, float radius, int material)
{
  Sphere s;
  glm_vec4(center, 0.0, s.center);
  s.radius = radius;
  s.matIdx = material;
  return s;
}

void triangleVertex(const float* vertices, int nVert, vec3 v)
{
  glm_vec3_copy((vec3){vertices[nVert*3],vertices[nVert*3+1],vertices[nVert*3+2]}, v);
}

void triangle(Triangle* t, vec3 v0, vec3 v1, vec3 v2, int matIdx)
{
  vec3 v10;
  vec3 v20;

  glm_vec3_sub(v1, v0, v10);
  glm_vec3_sub(v2, v0, v20);

  glm_vec3_cross(v10, v20, t->normal);
  glm_vec3_normalize(t->normal);

  glm_vec3_add(v0, v1, t->centroid);
  glm_vec3_add(v2, t->centroid, t->centroid);
  glm_vec3_divs(t->centroid, 3, t->centroid);

  t->matIdx = matIdx;
}


Material* genMaterialBuffer(int count)
{
  Material* materials = malloc(count * sizeof(Material));
  return materials;
}

Sphere* genSphereBuffer(int count)
{
  Sphere* spheres = malloc(count * sizeof(Sphere));
  return spheres;
}

Triangle* genTriangleBuffer(int count)
{
  Triangle* triangles = malloc(count * sizeof(Triangle));
  return triangles;
}

int* genSphereIDs(int count)
{
  int* sphereIDs = malloc(count * sizeof(int));
  for (int i=0; i<count; i++) 
  {
    sphereIDs[i] = i;
  }
  return sphereIDs;
}
