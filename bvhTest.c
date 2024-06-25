#include "src/bvh.h"
#include "src/nodes.h"
#include "src/hittable.h"

int main(void)
{
  Sphere* spheres = calloc(4, sizeof(Sphere));

  Material m = dielectric(1.5);
  
  spheres[0] = sphere((vec3){-3,0.2,-3}, 0.2, m);
  spheres[1] = sphere((vec3){-2,0.3,1.5}, 0.3, m);
  spheres[2] = sphere((vec3){1,0.2,-4}, 0.2, m);
  spheres[3] = sphere((vec3){3,0.3,2}, 0.3, m);
  // spheres[4] = sphere((vec3){2.1,0.3,-0.5}, 0.3, m);
  // spheres[5] = sphere((vec3){-0.7,0.2,0.9}, 0.2, m);

  int* sphereIDs = genSphereIDs(4);
  
  Node* nodes = makeNodes(2*6+1);
  
  int nodesUsed = buildBVH(nodes, spheres, sphereIDs, 4, 4);
  printf("\nNodes used:%d\n",nodesUsed);
  printNodes(nodes,nodesUsed);

  for(int i = 0; i < 4; i++)
  {
    printf("| %d |",sphereIDs[i]);
  }
  printf("\n");

  return 0;
}
