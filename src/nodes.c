#include "aabb.h"
#include "cglm/types.h"

AABB* makeAABBs(int nPrimitives)
{
  AABB* nodes = calloc(nPrimitives, sizeof(AABB));
  for (int i = 0; i < nPrimitives; i++)
  {
    glm_vec4((vec3){INFINITY, INFINITY, INFINITY}, 0.0, nodes[i].minB);
    nodes[i].nPrimitives = 0;
    glm_vec4((vec3){-INFINITY, -INFINITY, -INFINITY}, 0.0, nodes[i].maxB);
    nodes[i].primitiveOffset = -1;
  }

  return nodes;
}

void resetAABBs(AABB* nodes, int offset, int count)
{
  for(int i=0; i<count; i++)
  {
    glm_vec4((vec3){INFINITY, INFINITY, INFINITY}, 0.0, nodes[offset+i].minB);
    nodes[offset+i].nPrimitives = 0;
    glm_vec4((vec3){-INFINITY, -INFINITY, -INFINITY}, 0.0, nodes[offset+i].maxB);
    nodes[offset+i].primitiveOffset = -1;
  }
}

void printAABBs(AABB* nodes, int nNodes)
{
  for(int i=0; i<nNodes; i++)
  {
    printf("AABB: %d SplitAxis: %d\n"
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

float surfaceArea(AABB *node)
{
  vec3 diag;
  glm_vec3_sub(node->maxB, node->minB, diag);
  return 2 * (diag[0] * diag[1] + diag[0] * diag[2] + diag[1] * diag[2]);
}