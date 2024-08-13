#ifndef AABBS
#define AABBS

#include <stdlib.h>
#include <math.h>
#include "cglm/cglm.h"

typedef struct
{
  vec4 minB;
  vec4 maxB;
  int splitAxis;
  int nPrimitives;
  int primitiveOffset;
} AABB;

AABB* makeAABBs(int nPrimitives);
void resetAABBs(AABB* AABBs, int offset, int count);
void printAABBs(AABB* AABBs, int nAABBs);
float surfaceArea(AABB* AABB);

#endif
