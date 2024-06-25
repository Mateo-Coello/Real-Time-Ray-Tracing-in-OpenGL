#ifndef BVH_OBJECTS
#define BVH_OBJECTS

#include <cglm/cglm.h>
#include "math.h"
#include "nodes.h"
#include "hittable.h"

int buildBVH(Node* nodes, int nObjects, Primitive* objects, PrimitiveInfo* objectIDs, int nBins);
void updateBounds(Node* nodes, Primitive* objects, PrimitiveInfo* objectIDs, int nodeIdx);
int subdivide(Node* nodes, Primitive* objects, PrimitiveInfo* objectIDs, int nodeIdx, int nodesUsed, Node* bins, int nBins);
void determineBestSplitBin(Node* nodes, Primitive* objects, PrimitiveInfo* objectIDs, int nodeIdx, int axis, int binCount, Node* bins, float* bestPos, float* bestCost);
void buildBins(Node* nodes, Primitive* objects, PrimitiveInfo* objectIDs, int nodeIdx, int axis, int binCount, Node* bins);
void collectBins(Node* bins, int binCount);
int objectSplit(Node* nodes, Primitive* objects, PrimitiveInfo* objectIDs, int parentIdx, int axis, int nodesUsed, float splitPos, Node* bins, int nBins);
#endif
