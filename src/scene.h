#ifndef SCENE_H
#define SCENE_H

#include <stdio.h>
#include <stdlib.h>
#include <cglm/vec3.h>

#include "hittable.h"
#include "nodes.h"

#include <memory.h>
#include <stdbool.h>
#include <string.h>

#include "tinyobj_loader_c.h"

#include <float.h>
#include <limits.h>
#include <math.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

typedef struct
{
  Material* materials;
  Sphere* spheres;
  Triangle* triangles;
  Node* bvh;
  PrimitiveInfo* objectIDs;
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

int subdivide(Scene* s, int nodeIdx, int nodesUsed, Node* bins, int nBins);

void determineBestSplitBin(Scene* s, int nodeIdx, int axis, int binCount, Node* bins, float* bestPos, float* bestCost);

void buildBins(Scene* s, int nodeIdx, int axis, int binCount, Node* bins);

void collectBins(Node* bins, int binCount);

int objectSplit(Scene* s, int parentIdx, int axis, int nodesUsed, float splitPos, Node* bins, int nBins);

Scene generateSimpleScene();

Scene generateRocksScene();

Scene generateRainbowLightsScene();

Scene generateCornellBoxScene();

void deleteSceneResources(Scene* s);

void loadModel(Scene* s, const char* filename);

PrimitiveInfo* genObjectIDs(int nObjs[]);


#endif
