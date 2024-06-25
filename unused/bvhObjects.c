// #include "bvhObjects.h"


// int buildBVH(Scene* s, int nBins)
// {
//   s->bvh[0].count = s.nObjs;
//   s->bvh[0].content = 0;

//   Nodes bins = makeNodes(nBins*3 + 2);

//   updateBounds(nodes, objects, objectIDs, 0);
//   int nodesUsed = subdivide(nodes, objects, objectIDs, 0, 1, bins, nBins);

//   free(bins);
//   return nodesUsed;
// }

// void updateBounds(Node* nodes, Primitive* objects, PrimitiveInfo* objectIDs, int nodeIdx)
// {
//   int count = nodes[nodeIdx].count; // Number of spheres in node
//   int first = nodes[nodeIdx].content;

//   for(int i = 0; i < count; i++)
//   {
//     int objectIdx = objectIDs[first+i].idx;

//     vec3 minB, maxB;

//     // Sphere Primitive
//     if (objectIDs[first+i].type == SPHERE)
//     {
//         // find new minimum
//         glm_vec3_subs(objects[objectIdx].s.center, objects[objectIdx].s.radius, minB);
//         glm_vec3_minv(minB, nodes[nodeIdx].minB, nodes[nodeIdx].minB);
    
//         // find new maximum
//         glm_vec3_adds(objects[objectIdx].s.center, objects[objectIdx].s.radius, maxB);
//         glm_vec3_maxv(maxB, nodes[nodeIdx].maxB, nodes[nodeIdx].maxB);
//         continue;
//     }

//     // Triangle Primitive
//     // find new minimum
//     glm_vec3_minv(nodes[nodeIdx].minB, objects[objectIdx].t.a, nodes[nodeIdx].minB);
//     glm_vec3_minv(nodes[nodeIdx].minB, objects[objectIdx].t.b, nodes[nodeIdx].minB);
//     glm_vec3_minv(nodes[nodeIdx].minB, objects[objectIdx].t.c, nodes[nodeIdx].minB);

//     // find new maximum
//     glm_vec3_maxv(nodes[nodeIdx].maxB, objects[objectIdx].t.a, nodes[nodeIdx].maxB);
//     glm_vec3_maxv(nodes[nodeIdx].maxB, objects[objectIdx].t.b, nodes[nodeIdx].maxB);
//     glm_vec3_maxv(nodes[nodeIdx].maxB, objects[objectIdx].t.c, nodes[nodeIdx].maxB);
//   }
// }

// int subdivide(Node* nodes, Primitive* objects, PrimitiveInfo* objectIDs, int nodeIdx, int nodesUsed, Node* bins, int nBins)
// {
//   int objectCount = nodes[nodeIdx].count;

//   if (objectCount < 2) return nodesUsed;

//   vec3 dimensions;
//   glm_vec3_sub(nodes[nodeIdx].maxB, nodes[nodeIdx].minB, dimensions);
  
//   int bestAxis = 0;
//   float dimension = dimensions[0];
  
//   if (dimensions[1] > dimension)
//   {
//     bestAxis = 1;
//     dimension = dimensions[1];
//   } 
//   if (dimensions[2] > dimension) 
//   {
//     bestAxis = 2;
//     dimension = dimensions[2];
//   } 

//   int binCount = nBins;

//   float pos, cost;
//   determineBestSplitBin(nodes, objects, objectIDs, nodeIdx, bestAxis, binCount, bins, &pos, &cost);
//   // printf("\nDetermine Best Split Bin\n");
//   // printNodes(bins, 14);

//   float nodeCost = objectCount * (dimensions[0]*dimensions[1] + dimensions[1]*dimensions[2] + dimensions[0]*dimensions[2]);
//   // printf("Node cost:%f, cost:%f\n", nodeCost, cost);
//   // printf("Best position: %f\n", pos);
  
//   if (cost >= nodeCost) return nodesUsed;
    
//   return objectSplit(nodes, objects, objectIDs, nodeIdx, bestAxis, nodesUsed, pos, bins, nBins);
// }

// void determineBestSplitBin(Node* nodes, Primitive* objects, PrimitiveInfo* objectIDs, int nodeIdx, int axis, int binCount, Node* bins, float* bestPos, float* bestCost)
// {
//   resetNodes(bins, 0, binCount*3+2);
//   buildBins(nodes, objects, objectIDs, nodeIdx, axis, binCount, bins);
//   // printf("\nBuild Bins\n");
//   // printNodes(bins, 14);
//   collectBins(bins, binCount);
//   // printf("\nCollect Bins\n");
//   // printNodes(bins, 14);

//   *bestPos = 0;
//   *bestCost = FLT_MAX;
//   float testPos = nodes[nodeIdx].minB[axis];
//   float extent = nodes[nodeIdx].maxB[axis] - testPos;
//   float scale = extent/binCount;
  
//   // printf("Extent:%f Bin Size:%f\n", extent, scale);

//   for(int i = 0; i < binCount; i++)
//   {
//     int leftIdx = i+binCount;
//     int rightIdx = i+binCount*2;

//     // unpack left box
//     vec3 lB;
//     glm_vec3_sub(bins[leftIdx].maxB, bins[leftIdx].minB, lB);
//     int sphereCountLB = bins[leftIdx].count;
//     // printf("lB x:%f y:%f z:%f\n", lB[0], lB[1], lB[2]);
//     // unpack left box
//     vec3 rB;
//     glm_vec3_sub(bins[rightIdx].maxB, bins[rightIdx].minB, rB);
//     int sphereCountRB = bins[rightIdx].count;
//     // printf("rB x:%f y:%f z:%f\n", rB[0], rB[1], rB[2]);

//     float cost = sphereCountLB * (lB[0] * lB[1] + lB[1] * lB[2] + lB[0] * lB[2]) + sphereCountRB * (rB[0] * rB[1] + rB[1] * rB[2] + rB[0] * rB[2]) ;
//     // printf("Cost:%f\n", cost);
    
//     if (cost < *bestCost)
//     {
//       *bestPos = testPos + (i + 1) * scale;
//       *bestCost = cost;
//     }
//   }
// }

// void buildBins(Node* nodes, Primitive* objects, PrimitiveInfo* objectIDs, int nodeIdx, int axis, int binCount, Node* bins)
// {
//   resetNodes(bins, 0, binCount);

//   int objectCount = nodes[nodeIdx].count;
//   int first = nodes[nodeIdx].content;

//   float minB = nodes[nodeIdx].minB[axis];
//   float maxB = nodes[nodeIdx].maxB[axis];
  
//   float extent = maxB - minB; 
//   float binSize = extent/binCount;
  
//   for(int i = 0; i < objectCount; i++)
//   {
//     int objectIdx = objectIDs[first+i].idx;

//     // Sphere Primitive
//     if (objectIDs[first+i].type == SPHERE)
//     {
//       float sphereCenter = objects[objectIdx].s.center[axis];
//       float sphereOffset = sphereCenter - minB;
  
//       // printf("Sphere idx:%d, center: %f, offset: %f\n", sphereIdx, sphereCenter, sphereOffset);
//       vec3 sphereMaxB, sphereMinB;
//       glm_vec3_subs(objects[objectIdx].s.center, objects[objectIdx].s.radius, sphereMinB);
//       glm_vec3_adds(objects[objectIdx].s.center, objects[objectIdx].s.radius, sphereMaxB);
  
//       int binIdx = max(0, min(binCount-1, (int)sphereOffset/binSize));
//       // printf("Bin index: %d\n", binIdx);
//       glm_vec3_minv(bins[binIdx].minB, sphereMinB, bins[binIdx].minB);     
//       glm_vec3_maxv(bins[binIdx].maxB, sphereMaxB, bins[binIdx].maxB);
//       bins[binIdx].count ++;        
//       continue;
//     }

//     // Triangle Primitive
//     float offset = objects[objectIdx].t.centroid[axis] - minB;
//     int binIdx = max(0, min(binCount-1, (int)offset/binSize));
    
//     glm_vec3_minv(nodes[nodeIdx].minB, objects[objectIdx].t.a, nodes[nodeIdx].minB);
//     glm_vec3_minv(nodes[nodeIdx].minB, objects[objectIdx].t.b, nodes[nodeIdx].minB);
//     glm_vec3_minv(nodes[nodeIdx].minB, objects[objectIdx].t.c, nodes[nodeIdx].minB);

//     glm_vec3_maxv(nodes[nodeIdx].maxB, objects[objectIdx].t.a, nodes[nodeIdx].maxB);
//     glm_vec3_maxv(nodes[nodeIdx].maxB, objects[objectIdx].t.b, nodes[nodeIdx].maxB);
//     glm_vec3_maxv(nodes[nodeIdx].maxB, objects[objectIdx].t.c, nodes[nodeIdx].maxB);
//     bins[binIdx].count++;
//   }
// }

// void collectBins(Node* bins, int binCount)
// {
//   for(int i = 0; i < binCount; i++)
//   {
//     // Note
//     // bins[48] and bins[49] are used for temporary data

//     // Expand left box
//     int leftIdx = i + binCount;
    
//     glm_vec3_minv(bins[binCount*3].minB, bins[i].minB, bins[binCount*3].minB);
//     bins[binCount*3].count += bins[i].count;
//     glm_vec3_maxv(bins[binCount*3].maxB, bins[i].maxB, bins[binCount*3].maxB);

//     bins[leftIdx] = bins[binCount*3];

//     // Expand right box
//     int rightIdx = (binCount - 1 - i + binCount*2);
//     int binIdx = binCount - 1 - i;
//     glm_vec3_minv(bins[binCount*3+1].minB, bins[binIdx].minB, bins[binCount*3+1].minB);
//     bins[binCount*3+1].count += bins[binIdx].count;
//     glm_vec3_maxv(bins[binCount*3+1].maxB, bins[binIdx].maxB, bins[binCount*3+1].maxB);

//     bins[rightIdx] = bins[binCount*3+1];
//   }
// }

// int objectSplit(Node* nodes, Primitive* objects, PrimitiveInfo* objectIDs, int parentIdx, int axis, int nodesUsed, float splitPos, Node* bins, int nBins)
// {
//   int objectCount = nodes[parentIdx].count;
//   int contents = nodes[parentIdx].content;

//   // split groups
//   int i = contents;
//   int j = i + objectCount - 1;
//   PrimitiveInfo temp;
//   while (i <= j)
//   {
//     Primitive object = objects[objectIDs[i].idx];

//     i++;
//     if (objectIDs[i].type == SPHERE && object.s.center[axis] < splitPos)  continue;
//     else if (objectIDs[i].type == TRIANGLE && object.t.centroid[axis] < splitPos) continue;

//     i--;
//     temp = objectIDs[i];
//     objectIDs[i] = objectIDs[j];
//     objectIDs[j] = temp;
//     j--;
//   }

//   // create child nodes
//   int leftCount = i - contents;

//   if (leftCount == 0 || leftCount == objectCount) return nodesUsed;

//   int leftChildIdx = nodesUsed;
//   nodesUsed ++;
//   nodes[leftChildIdx].content = contents;
//   nodes[leftChildIdx].count = leftCount;
//   nodes[parentIdx].content = leftChildIdx;

//   int rightChildIdx = nodesUsed;
//   nodesUsed ++;
//   nodes[rightChildIdx].content = i;
//   nodes[rightChildIdx].count = objectCount - leftCount;

//   updateBounds(nodes, objects, objectIDs, leftChildIdx);
//   nodesUsed = subdivide(nodes, objects, objectIDs, leftChildIdx, nodesUsed, bins, nBins);
//   updateBounds(nodes, objects, objectIDs, rightChildIdx);
//   nodesUsed = subdivide(nodes, objects, objectIDs, rightChildIdx, nodesUsed, bins, nBins);

//   nodes[parentIdx].count = 0;
//   return nodesUsed;
// }