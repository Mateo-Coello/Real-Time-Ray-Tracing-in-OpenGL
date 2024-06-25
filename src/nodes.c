#include "nodes.h"
#include <cglm/types.h>

Nodes makeNodes(int nObjs)
{
  Nodes nodes = calloc(nObjs, sizeof(Node));
  for (int i = 0; i < nObjs; i++)
  {
    glm_vec4((vec3){INFINITY, INFINITY, INFINITY}, 0.0, nodes[i].minB);
    nodes[i].count = 0;
    glm_vec4((vec3){-INFINITY, -INFINITY, -INFINITY}, 0.0, nodes[i].maxB);
    nodes[i].content = -1;
  }

  return nodes;
}

void resetNodes(Nodes nodes, int offset, int count)
{
  for(int i=0; i<count; i++)
  {
    glm_vec4((vec3){INFINITY, INFINITY, INFINITY}, 0.0, nodes[offset+i].minB);
    nodes[offset+i].count = 0;
    glm_vec4((vec3){-INFINITY, -INFINITY, -INFINITY}, 0.0, nodes[offset+i].maxB);
    nodes[offset+i].content = -1;
  }
}

void printNodes(Nodes nodes, int nNodes)
{
  for(int i=0; i<nNodes; i++)
  {
    printf("Node %d\n"
           "Min Bound x:%f y:%f z:%f\n"
           "Max Bound x:%f y:%f z:%f\n"
           "Count: %d Content:%d\n", 
            i,
            nodes[i].minB[0],nodes[i].minB[1],nodes[i].minB[2],
            nodes[i].maxB[0],nodes[i].maxB[1],nodes[i].maxB[2],
            nodes[i].count, nodes[i].content
            );
  }
}
