#include "nodes.h"
#include <cglm/types.h>

Nodes makeNodes(int nPrimitives)
{
  Nodes nodes = calloc(nPrimitives, sizeof(Node));
  for (int i = 0; i < nPrimitives; i++)
  {
    glm_vec4((vec3){INFINITY, INFINITY, INFINITY}, 0.0, nodes[i].minB);
    nodes[i].nPrimitives = 0;
    glm_vec4((vec3){-INFINITY, -INFINITY, -INFINITY}, 0.0, nodes[i].maxB);
    nodes[i].primitiveOffset = -1;
  }

  return nodes;
}

void resetNodes(Nodes nodes, int offset, int count)
{
  for(int i=0; i<count; i++)
  {
    glm_vec4((vec3){INFINITY, INFINITY, INFINITY}, 0.0, nodes[offset+i].minB);
    nodes[offset+i].nPrimitives = 0;
    glm_vec4((vec3){-INFINITY, -INFINITY, -INFINITY}, 0.0, nodes[offset+i].maxB);
    nodes[offset+i].primitiveOffset = -1;
  }
}

void printNodes(Nodes nodes, int nNodes)
{
  for(int i=0; i<nNodes; i++)
  {
    printf("Node: %d SplitAxis: %d\n"
           "Min Bound x:%f y:%f z:%f\n"
           "Max Bound x:%f y:%f z:%f\n"
           "Num Primitives: %d Primitive Offset:%d\n", 
            i, nodes[i].splitAxis,
            nodes[i].minB[0],nodes[i].minB[1],nodes[i].minB[2],
            nodes[i].maxB[0],nodes[i].maxB[1],nodes[i].maxB[2],
            nodes[i].nPrimitives, nodes[i].primitiveOffset
            );
  }
}
