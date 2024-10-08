#ifndef SCENE_H
#define SCENE_H

#if defined(_WIN32) || defined(_WIN64)
#define IS_WINDOWS
#elif defined(__linux__)
#define IS_LINUX
#else
#error "Unknown operating system"
#endif

#include <stdio.h>
#include <stdlib.h>
#include "cglm/vec3.h"

#include "hittable.h"
#include "aabb.h"
#include "tinyobj_loader_c.h"
#include "random.h"

#include <memory.h>
#include <stdbool.h>
#include <string.h>

#include <float.h>
#include <limits.h>
#include <math.h>

#include <fcntl.h>

#if defined(IS_LINUX)
#include <sys/mman.h>
#endif

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

typedef struct
{
  Material* materials;
  Sphere* spheres;
  Triangle* triangles;
  AABB* bvh;
  AABB* primitivesAABB;
  PrimitiveInfo* primitiveIDs;
  tinyobj_attrib_t attrib;
  tinyobj_shape_t* shapes;
  tinyobj_material_t* mats;
  size_t nShapes;
  size_t nMats;
  int nMaterials;
  int nObjs[4];
  int nodesUsed;
} Scene;

void buildBVH(Scene* s, int nBins);

void updateBounds(Scene* s, int nodeIdx);

int subdivide(Scene* s, int nodeIdx, int nodesUsed, AABB* bins, int nBins);

void determineBestSplitBin(Scene* s, int nodeIdx, int axis, int binCount, AABB* bins, float* bestPos, float* bestCost);

void buildBins(Scene* s, int nodeIdx, int axis, int binCount, AABB* bins);

void computePossibleBinPartitions(AABB* bins, int binCount);

int splitAABB(Scene* s, int parentIdx, int axis, int nodesUsed, float splitPos, AABB* bins, int nBins);

Scene generateSceneModel(char *modelName);

Scene generateDefaultScene();

void deleteSceneResources(Scene* s);

void loadModel(Scene* s, const char* filename);

PrimitiveInfo* genObjectIDs(int nObjs[]);


#endif