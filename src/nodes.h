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
  int count;
  int content;
} Node;

typedef Node* Nodes;

Nodes makeNodes(int nObjs);
void resetNodes(Nodes nodes, int offset, int count);
void printNodes(Nodes nodes, int nNodes);

#endif
