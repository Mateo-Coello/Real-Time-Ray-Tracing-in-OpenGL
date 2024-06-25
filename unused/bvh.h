#ifndef BVH
#define BVH

#include <cglm/cglm.h>
#include "math.h"
#include "nodes.h"
#include "hittable.h"

int buildBVH(Node* nodes, Sphere* spheres, int* sphereIds, int sphereCount, int nBins);
void updateBounds(Nodes nodes, Sphere* spheres, int* sphereIds, int nodeIdx);
int subdivide(Node* nodes, Sphere* spheres, int* sphereIDs, int nodeIdx, int nodesUsed, Node* bins, int nBins);
void determineBestSplitBin(Node* nodes, Sphere* spheres, int* sphereIds, int nodeIdx, int axis, int binCount, Node* bins, float* pos, float* cost);
void buildBins(Node* nodes, Sphere* spheres, int* sphereIds, int nodeIdx, int axis, int binCount, Node* bins);
void collectBins(Node* nodes, int binCount);
int objectSplit(Node* nodes, Sphere* spheres, int* sphereIds, int parentIdx, int axis, int nodesUsed, float splitPos, Node* bins, int nBins);

#endif
