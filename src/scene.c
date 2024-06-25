#include <cglm/vec3.h>
#define TINYOBJ_LOADER_C_IMPLEMENTATION
#include "tinyobj_loader_c.h"
#include "scene.h"

int max(int a, int b){return a > b ? a : b;}
int min(int a, int b){return a < b ? a : b;}

Scene generateCornellBoxScene()
{
  Scene s = {};
  
  s.nObjs[0] = 2; // num spheres
  s.nObjs[3] = 1; // num lights
  s.spheres = genSphereBuffer(s.nObjs[0]);

  s.spheres[0] = sphere((vec3){0.0, 0.55, 0.0}, 0.5, 0);
  s.spheres[1] = sphere((vec3){0.0, 8.0, 0.0}, 0.02, 3);
  
  // Scene Triangles
  loadModel(&s, "/home/mai/Documents/C/rt_pbr/models/cornellbox.obj");
    
  // Scene BVH
  s.objectIDs = genObjectIDs(s.nObjs);
  s.bvh = makeNodes(2*s.nObjs[2]+1);
  int nBins = 10;
  buildBVH(&s, nBins);

  // for(int i = 0; i < s.nObjs[2]; i++)
  // {
  //   printf("| %d |", s.objectIDs[i].idx);
  // }
  // printf("\n");
  // for(int i = 0; i < s.nObjs[2]; i++)
  // {
  //   printf("| %c |", s.objectIDs[i].type == SPHERE ? 'S' : 'T');
  // }
  // printf("\n");
  // for(int i=0; i < s.nObjs[1]; i++)
  // {
  //   printf("%d %d %d\n", s.triangles[i].a, s.triangles[i].b, s.triangles[i].c);
  // }  
  return s;
}

Scene generateRocksScene()
{
  Scene s = {};

  loadModel(&s, "/home/mai/Documents/C/computeShader/models/rocks.obj");
    
  s.nMaterials = s.nMats;
  s.materials = genMaterialBuffer(s.nMaterials);
  
  int nBins = 12;
  buildBVH(&s, nBins);

  printf("Num triangles:%d\n",s.nObjs[2]);
  
  printf("%ld\n", s.nShapes);
  printf("%d\n", s.attrib.num_faces);
  printf("%d\n", s.attrib.num_face_num_verts);
  printf("%ld\n", s.nMats);
  
  // for(int i=0; i < s.nMats; i++)
  // {
  //   s.materials[i] = lambertian(s.mats[i].diffuse);
  //   printf("%f %f %f\n", s.materials[i].albedo[0],s.materials[i].albedo[1],s.materials[i].albedo[2]);
  // }
  
  return s;
}

Scene generateRainbowLightsScene()
{
  Scene s = {};

  s.nObjs[0] = 3;
  s.spheres = genSphereBuffer(s.nObjs[0]);

  s.spheres[0] = sphere((vec3){0.0, 0.5, 0.0}, 0.5, 0);
  s.spheres[1] = sphere((vec3){0.7, 0.2, 0.3}, 0.2, 2);
  s.spheres[2] = sphere((vec3){-0.6, 0.1, 0.55}, 0.1, 1);

  loadModel(&s, "/home/mai/Documents/C/computeShader/models/rainbow.obj");

  s.nMaterials = s.nMats;
  s.materials = genMaterialBuffer(s.nMaterials);
  
  printf("%d\n",s.nMaterials);

  int nBins = 12;
  buildBVH(&s, nBins);

  // for(int i=0; i < 2; i++)
  // {
  //   s.materials[i] = lambertian(s.mats[i].diffuse);
  // }

  // s.materials[2] = metal(s.mats[2].diffuse, 0.0);

  // for(int i=3; i < s.nMats; i++)
  // {
  //   s.materials[i] = emissive(s.mats[i].diffuse, 1);
  //   printf("%f %f %f\n", s.materials[i].albedo[0],s.materials[i].albedo[1],s.materials[i].albedo[2]);
  // }
  
  return s;
}
  
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//               Resource Functions
// ----------------------------------------------

void deleteSceneResources(Scene* s)
{
  if (s->spheres != NULL) free(s->spheres);
  if (s->attrib.vertices != NULL) tinyobj_attrib_free(&s->attrib);
  if (s->shapes != NULL) tinyobj_shapes_free(s->shapes, s->nShapes);
  if (s->mats != NULL) tinyobj_materials_free(s->mats, s->nMats);
  if (s->triangles != NULL) free(s->triangles);
  free(s->materials);
  free(s->bvh);
  free(s->objectIDs);
}


// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                 BVH Functions
// ----------------------------------------------
void buildBVH(Scene* s, int nBins)
{
  s->objectIDs = genObjectIDs(s->nObjs);
  s->bvh = makeNodes(2*s->nObjs[2]+1);
  
  s->bvh[0].nPrimitives = s->nObjs[2];
  s->bvh[0].primitiveOffset = 0;

  Node* bins = makeNodes(nBins*3 + 2);

  updateBounds(s, 0);
  
  s->nodesUsed = subdivide(s, 0, 1, bins, nBins);
  s->bvh = realloc(s->bvh, s->nodesUsed*sizeof(Node));
  
  free(bins);
}

void updateBounds(Scene* s, int nodeIdx)
{
  int count = s->bvh[nodeIdx].nPrimitives; 
  int first = s->bvh[nodeIdx].primitiveOffset;

  for(int i = 0; i < count; i++)
  {
    PrimitiveInfo object = s->objectIDs[first+i];
    vec3 minB, maxB;

    // Sphere Primitive
    if (object.type == SPHERE)
    {
        // find new minimum
        glm_vec3_subs(s->spheres[object.idx].center, s->spheres[object.idx].radius, minB);
        glm_vec3_minv(minB, s->bvh[nodeIdx].minB, s->bvh[nodeIdx].minB);
    
        // find new maximum
        glm_vec3_adds(s->spheres[object.idx].center, s->spheres[object.idx].radius, maxB);
        glm_vec3_maxv(maxB, s->bvh[nodeIdx].maxB, s->bvh[nodeIdx].maxB);
        continue;
    }

    // Triangle Primitive
    vec3 a, b, c; 
    triangleVertex(s->attrib.vertices, s->triangles[object.idx].a, a);
    triangleVertex(s->attrib.vertices, s->triangles[object.idx].b, b);
    triangleVertex(s->attrib.vertices, s->triangles[object.idx].c, c);
    
    // find new minimum
    glm_vec3_minv(s->bvh[nodeIdx].minB, a, s->bvh[nodeIdx].minB);
    glm_vec3_minv(s->bvh[nodeIdx].minB, b, s->bvh[nodeIdx].minB);
    glm_vec3_minv(s->bvh[nodeIdx].minB, c, s->bvh[nodeIdx].minB);

    // find new maximum
    glm_vec3_maxv(s->bvh[nodeIdx].maxB, a, s->bvh[nodeIdx].maxB);
    glm_vec3_maxv(s->bvh[nodeIdx].maxB, b, s->bvh[nodeIdx].maxB);
    glm_vec3_maxv(s->bvh[nodeIdx].maxB, c, s->bvh[nodeIdx].maxB);
  }
}

int subdivide(Scene* s, int nodeIdx, int nodesUsed, Node* bins, int nBins)
{
  int objectCount = s->bvh[nodeIdx].nPrimitives;

  if (objectCount < 2) return nodesUsed;

  vec3 dimensions;
  glm_vec3_sub(s->bvh[nodeIdx].maxB, s->bvh[nodeIdx].minB, dimensions);
  
  int bestAxis = glm_vec3_max(dimensions);
  s->bvh[nodeIdx].splitAxis = bestAxis;
  int binCount = nBins;

  float pos, cost;
  determineBestSplitBin(s, nodeIdx, bestAxis, binCount, bins, &pos, &cost);
  // printf("\nDetermine Best Split Bin\n");
  // printNodes(bins, 14);

  float nodeCost = objectCount * (dimensions[0]*dimensions[1] + dimensions[1]*dimensions[2] + dimensions[0]*dimensions[2]);
  // printf("Node cost:%f, cost:%f\n", nodeCost, cost);
  // printf("Best position: %f\n", pos);
  
  if (cost >= nodeCost) return nodesUsed;
    
  return objectSplit(s, nodeIdx, bestAxis, nodesUsed, pos, bins, nBins);
}

void determineBestSplitBin(Scene* s, int nodeIdx, int axis, int binCount, Node* bins, float* bestPos, float* bestCost)
{
  resetNodes(bins, 0, binCount*3+2);
  buildBins(s, nodeIdx, axis, binCount, bins);
  // printf("\nBuild Bins\n");
  // printNodes(bins, 14);
  collectBins(bins, binCount);
  // printf("\nCollect Bins\n");
  // printNodes(bins, 14);

  *bestPos = 0;
  *bestCost = FLT_MAX;
  float testPos = s->bvh[nodeIdx].minB[axis];
  float extent = s->bvh[nodeIdx].maxB[axis] - testPos;
  float scale = extent/binCount;
  
  // printf("Extent:%f Bin Size:%f\n", extent, scale);

  for(int i = 0; i < binCount; i++)
  {
    int leftIdx = i+binCount;
    int rightIdx = i+binCount*2;

    // unpack left box
    vec3 lB;
    glm_vec3_sub(bins[leftIdx].maxB, bins[leftIdx].minB, lB);
    int objectCountLB = bins[leftIdx].nPrimitives;
    // printf("lB x:%f y:%f z:%f\n", lB[0], lB[1], lB[2]);
    // unpack left box
    vec3 rB;
    glm_vec3_sub(bins[rightIdx].maxB, bins[rightIdx].minB, rB);
    int objectCountRB = bins[rightIdx].nPrimitives;
    // printf("rB x:%f y:%f z:%f\n", rB[0], rB[1], rB[2]);

    float cost = objectCountLB * (lB[0] * lB[1] + lB[1] * lB[2] + lB[0] * lB[2]) + objectCountRB * (rB[0] * rB[1] + rB[1] * rB[2] + rB[0] * rB[2]) ;
    // printf("Cost:%f\n", cost);
    
    if (cost < *bestCost)
    {
      *bestPos = testPos + (i + 1) * scale;
      *bestCost = cost;
    }
  }
}

void buildBins(Scene* s, int nodeIdx, int axis, int binCount, Node* bins)
{
  resetNodes(bins, 0, binCount);

  int objectCount = s->bvh[nodeIdx].nPrimitives;
  int first = s->bvh[nodeIdx].primitiveOffset;

  float minB = s->bvh[nodeIdx].minB[axis];
  float maxB = s->bvh[nodeIdx].maxB[axis];
  
  float extent = maxB - minB; 
  float binSize = extent/binCount;
  
  for(int i = 0; i < objectCount; i++)
  {
    PrimitiveInfo object = s->objectIDs[first+i];

    // Sphere Primitive
    if (object.type == SPHERE)
    {
      float sphereCenter = s->spheres[object.idx].center[axis];
      float sphereOffset = sphereCenter - minB;
  
      // printf("Sphere idx:%d, center: %f, offset: %f\n", sphereIdx, sphereCenter, sphereOffset);
      vec3 sphereMaxB, sphereMinB;
      glm_vec3_subs(s->spheres[object.idx].center, s->spheres[object.idx].radius, sphereMinB);
      glm_vec3_adds(s->spheres[object.idx].center, s->spheres[object.idx].radius, sphereMaxB);
  
      int binIdx = max(0, min(binCount-1, (int)sphereOffset/binSize));
      // printf("Bin index: %d\n", binIdx);
      glm_vec3_minv(bins[binIdx].minB, sphereMinB, bins[binIdx].minB);     
      glm_vec3_maxv(bins[binIdx].maxB, sphereMaxB, bins[binIdx].maxB);
      bins[binIdx].nPrimitives ++;        
      continue;
    }

    // Triangle Primitive
    float offset = s->triangles[object.idx].centroid[axis] - minB;
    int binIdx = max(0, min(binCount-1, (int)offset/binSize));

    vec3 a, b, c;
    triangleVertex(s->attrib.vertices, s->triangles[object.idx].a, a);
    triangleVertex(s->attrib.vertices, s->triangles[object.idx].b, b);
    triangleVertex(s->attrib.vertices, s->triangles[object.idx].c, c);
    
    glm_vec3_minv(bins[binIdx].minB, a, bins[binIdx].minB);
    glm_vec3_minv(bins[binIdx].minB, b, bins[binIdx].minB);
    glm_vec3_minv(bins[binIdx].minB, c, bins[binIdx].minB);

    glm_vec3_maxv(bins[binIdx].maxB, a, bins[binIdx].maxB);
    glm_vec3_maxv(bins[binIdx].maxB, b, bins[binIdx].maxB);
    glm_vec3_maxv(bins[binIdx].maxB, c, bins[binIdx].maxB);
    bins[binIdx].nPrimitives++;
  }
}

void collectBins(Node* bins, int binCount)
{
  for(int i = 0; i < binCount; i++)
  {
    // Note
    // bins[48] and bins[49] are used for temporary data

    // Expand left box
    int leftIdx = i + binCount;
    
    glm_vec3_minv(bins[binCount*3].minB, bins[i].minB, bins[binCount*3].minB);
    bins[binCount*3].nPrimitives += bins[i].nPrimitives;
    glm_vec3_maxv(bins[binCount*3].maxB, bins[i].maxB, bins[binCount*3].maxB);

    bins[leftIdx] = bins[binCount*3];

    // Expand right box
    int rightIdx = (binCount - 1 - i + binCount*2);
    int binIdx = binCount - 1 - i;
    glm_vec3_minv(bins[binCount*3+1].minB, bins[binIdx].minB, bins[binCount*3+1].minB);
    bins[binCount*3+1].nPrimitives += bins[binIdx].nPrimitives;
    glm_vec3_maxv(bins[binCount*3+1].maxB, bins[binIdx].maxB, bins[binCount*3+1].maxB);

    bins[rightIdx] = bins[binCount*3+1];
  }
}

int objectSplit(Scene* s, int parentIdx, int axis, int nodesUsed, float splitPos, Node* bins, int nBins)
{
  int objectCount = s->bvh[parentIdx].nPrimitives;
  int contents = s->bvh[parentIdx].primitiveOffset;

  // split groups
  int i = contents;
  int j = i + objectCount - 1;
  PrimitiveInfo temp;
  while (i <= j)
  {
    if (s->objectIDs[i].type == SPHERE && s->spheres[s->objectIDs[i].idx].center[axis] < splitPos)  i++;
    else if (s->objectIDs[i].type == TRIANGLE && s->triangles[s->objectIDs[i].idx].centroid[axis] < splitPos) i++;
    else {
      temp = s->objectIDs[i];
      s->objectIDs[i] = s->objectIDs[j];
      s->objectIDs[j] = temp;
      j--;
    }
  }

  // create child nodes
  int leftCount = i - contents;

  if (leftCount == 0 || leftCount == objectCount) return nodesUsed;

  int leftChildIdx = nodesUsed;
  nodesUsed ++;
  s->bvh[leftChildIdx].primitiveOffset = contents;
  s->bvh[leftChildIdx].nPrimitives = leftCount;
  s->bvh[parentIdx].primitiveOffset = leftChildIdx;

  int rightChildIdx = nodesUsed;
  nodesUsed ++;
  s->bvh[rightChildIdx].primitiveOffset = i;
  s->bvh[rightChildIdx].nPrimitives = objectCount - leftCount;

  updateBounds(s, leftChildIdx);
  nodesUsed = subdivide(s, leftChildIdx, nodesUsed, bins, nBins);
  updateBounds(s, rightChildIdx);
  nodesUsed = subdivide(s, rightChildIdx, nodesUsed, bins, nBins);

  s->bvh[parentIdx].nPrimitives = 0;
  return nodesUsed;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                  Model Loading
// ----------------------------------------------

static char* mmap_file(size_t* len, const char* filename) {

  struct stat sb;
  char* p;
  int fd;

  fd = open(filename, O_RDONLY);
  if (fd == -1) {
    perror("open");
    return NULL;
  }

  if (fstat(fd, &sb) == -1) {
    perror("fstat");
    return NULL;
  }

  if (!S_ISREG(sb.st_mode)) {
    fprintf(stderr, "%s is not a file\n", filename);
    return NULL;
  }

  p = (char*)mmap(0, sb.st_size, PROT_READ, MAP_SHARED, fd, 0);

  if (p == MAP_FAILED) {
    perror("mmap");
    return NULL;
  }

  if (close(fd) == -1) {
    perror("close");
    return NULL;
  }

  (*len) = sb.st_size;

  return p;

}

static void get_file_data(void* ctx, const char* filename, const int is_mtl,
                          const char* obj_filename, char** data, size_t* len) {
  // NOTE: If you allocate the buffer with malloc(),
  // You can define your own memory management struct and pass it through `ctx`
  // to store the pointer and free memories at clean up stage(when you quit an
  // app)
  // This example uses mmap(), so no free() required.
  (void)ctx;

  if (!filename) {
    fprintf(stderr, "null filename\n");
    (*data) = NULL;
    (*len) = 0;
    return;
  }

  size_t data_len = 0;

  *data = mmap_file(&data_len, filename);
  (*len) = data_len;
}

void loadModel(Scene* s, const char* filename)
{  
  unsigned int flags = TINYOBJ_FLAG_TRIANGULATE;
  
  int ret =
      tinyobj_parse_obj(&s->attrib, &s->shapes, &s->nShapes, &s->mats,
                        &s->nMats, filename, get_file_data, NULL, flags);

  if (ret != TINYOBJ_SUCCESS) {
    printf("Error loading obj model\n");
    return;
  }
  s->nObjs[1] = s->attrib.num_face_num_verts; // num triangles

  int v0Idx, v1Idx, v2Idx;

  printf("%ld\n", s->nMats);
  s->materials = genMaterialBuffer(s->nMats);
  for (size_t m = 0; m < s->nMats; m++) {
    s->materials[m] = material(s->mats[m]);
    printf("%f %f %f\n", s->materials[m].albedo[0],s->materials[m].albedo[1],s->materials[m].albedo[2]);
    printf("%f %f %f\n", s->materials[m].specular[0],s->materials[m].specular[1],s->materials[m].specular[2]);
    printf("%f %f %f\n", s->materials[m].emission[0],s->materials[m].emission[1],s->materials[m].emission[2]);
    printf("%f %f \n", s->materials[m].roughness,s->materials[m].metalness);
  }
  
  s->triangles = malloc(sizeof(Triangle)*s->nObjs[1]);
  for (size_t f = 0; f < s->attrib.num_faces; f+=3)
  {    
    v0Idx = s->triangles[f/3].a = s->attrib.faces[ f ].v_idx;
    v1Idx = s->triangles[f/3].b = s->attrib.faces[f+1].v_idx;
    v2Idx = s->triangles[f/3].c = s->attrib.faces[f+2].v_idx;
    
    vec3 v0 = {s->attrib.vertices[v0Idx*3], s->attrib.vertices[v0Idx*3+1], s->attrib.vertices[v0Idx*3+2]};
    vec3 v1 = {s->attrib.vertices[v1Idx*3], s->attrib.vertices[v1Idx*3+1], s->attrib.vertices[v1Idx*3+2]}; 
    vec3 v2 = {s->attrib.vertices[v2Idx*3], s->attrib.vertices[v2Idx*3+1], s->attrib.vertices[v2Idx*3+2]};
    
    triangle(&s->triangles[f/3], v0, v1, v2, s->attrib.material_ids[f/3]);

    // printf("V%d:{%f,%f,%f} V%d:{%f,%f,%f} V%d:{%f,%f,%f}\n",
    //        v0Idx, s->attrib.vertices[v0Idx*3], s->attrib.vertices[v0Idx*3+1], s->attrib.vertices[v0Idx*3+2], 
    //        v1Idx, s->attrib.vertices[v1Idx*3], s->attrib.vertices[v1Idx*3+1], s->attrib.vertices[v1Idx*3+2], 
    //        v2Idx, s->attrib.vertices[v2Idx*3], s->attrib.vertices[v2Idx*3+1], s->attrib.vertices[v2Idx*3+2]);
  }
}

PrimitiveInfo* genObjectIDs(int nObjs[])
{
  nObjs[2] = nObjs[0] + nObjs[1]; // nSpheres + nTriangles
  PrimitiveInfo* objects = malloc((nObjs[2]) * sizeof(PrimitiveInfo));
  
  for (int i = 0; i < nObjs[0]; i++) 
  {
    objects[i].idx = i;
    objects[i].type = SPHERE;
  }
  
  for (int i = nObjs[0]; i < nObjs[2]; i++) 
  {
    objects[i].idx = i - nObjs[0];
    objects[i].type = TRIANGLE;
  }
  
  return objects;
}
