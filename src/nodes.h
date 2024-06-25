#ifndef NODES
#define NODES

#include <stdlib.h>
#include <math.h>
#include <cglm/cglm.h>

typedef struct
{
  vec4 minB;
  vec4 maxB;
  int splitAxis;
  int nPrimitives;
  int primitiveOffset;
} Node;

typedef Node* Nodes;

Nodes makeNodes(int nPrimitives);
void resetNodes(Nodes nodes, int offset, int nPrimitives);
void printNodes(Nodes nodes, int nNodes);

#endif
