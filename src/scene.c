#include <cglm/vec3.h>
#define TINYOBJ_LOADER_C_IMPLEMENTATION
#include "tinyobj_loader_c.h"
#include "scene.h"

int max(int a, int b){return a > b ? a : b;}
int min(int a, int b){return a < b ? a : b;}

Scene generateDefaultScene()
{
  Scene s = {};

  // Scene Spheres
  s.nObjs[0] = 1;
  s.spheres = genSphereBuffer(s.nObjs[0]);
  s.spheres[0] = sphere((vec3){0.0, 1.0, 0.0}, 0.5, 3);
  
  // Scene Triangles
  loadModel(&s, "./models/cornellbox.obj");
    
  // Scene BVH
  s.primitiveIDs = genObjectIDs(s.nObjs);
  int nBins = 11;
  buildBVH(&s, nBins);
  return s;
}

Scene generateSceneModel(char *modelName)
{
  Scene s = {};
  
  char path[100] = "./models/";
  strcat(path,modelName);
  loadModel(&s, path);

  s.primitiveIDs = genObjectIDs(s.nObjs);
  int nBins = 11;
  buildBVH(&s, nBins);
  
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
  free(s->primitiveIDs);
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                 BVH Construction
// ----------------------------------------------
void buildBVH(Scene* s, int nBins)
{
  s->primitiveIDs = genObjectIDs(s->nObjs);
  s->bvh = makeAABBs(2*s->nObjs[2]+1);
  
  s->bvh[0].nPrimitives = s->nObjs[2];
  s->bvh[0].primitiveOffset = 0;

  AABB* bins = makeAABBs(nBins*3 + 2);

  updateBounds(s, 0);
  
  s->nodesUsed = subdivide(s, 0, 1, bins, nBins);
  s->bvh = realloc(s->bvh, s->nodesUsed*sizeof(AABB));
  
  // printNodes(s->bvh, s->nodesUsed);
  free(bins);
}

// Compute or update the bounds of a node's AABB
void updateBounds(Scene* s, int nodeIdx)
{
  int count = s->bvh[nodeIdx].nPrimitives; // Number of spheres in node
  int first = s->bvh[nodeIdx].primitiveOffset;

  for(int i = 0; i < count; i++)
  {
    PrimitiveInfo object = s->primitiveIDs[first+i];
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

/* Check the cost of a partition based on the Surface Area Heurictic.
   If the cost of partitioning is better than creating a leaf then
   the node will be subdivided. */
int subdivide(Scene* s, int nodeIdx, int nodesUsed, AABB* bins, int nBins)
{
  int nPrimitives = s->bvh[nodeIdx].nPrimitives;
  
  if (nPrimitives < 2) return nodesUsed;

  // Determine the largest dimension of an AABB for selecting the split axis
  vec3 dimensions;
  glm_vec3_sub(s->bvh[nodeIdx].maxB, s->bvh[nodeIdx].minB, dimensions);

  int bestAxis = 0;
  float dimension = dimensions[0];

  if (dimensions[1] > dimension)
  {
    bestAxis = 1;
    dimension = dimensions[1];
  } 
  if (dimensions[2] > dimension) bestAxis = 2;
  
  s->bvh[nodeIdx].splitAxis = bestAxis;

  // Computer the cost of every possible bin partition along the largest axis
  float pos, cost;

  determineBestSplitBin(s, nodeIdx, bestAxis, nBins, bins, &pos, &cost);

  // float nodeCost = nPrimitives * surfaceArea(&s->bvh[nodeIdx]);
  float leafCost = (float)nPrimitives;
  
  if (cost >= leafCost) return nodesUsed;
    
  return splitAABB(s, nodeIdx, bestAxis, nodesUsed, pos, bins, nBins);
}

// Determine which possible partition of the bins is the best
void determineBestSplitBin(Scene* s, int nodeIdx, int axis, int nBins, AABB* bins, float* bestPos, float* bestCost)
{
  resetAABBs(bins, 0, nBins*3+2); // Clear bins content from previous computation
  buildBins(s, nodeIdx, axis, nBins, bins);
  computePossibleBinPartitions(bins, nBins);

  *bestPos = 0;
  *bestCost = FLT_MAX;
  
  float testPos = s->bvh[nodeIdx].minB[axis];
  float extent = s->bvh[nodeIdx].maxB[axis] - testPos;
  float scale = extent/nBins;

  // Determine which possible partition has the best cost
  for(int i = 0; i < nBins - 1; i++)
  {
    int leftIdx = i+nBins; // possible left partition
    int rightIdx = i+nBins*2; // possible right partition

    // Number of primitives in the left partition
    int objectCountLB = bins[leftIdx].nPrimitives;
    // Number of primitives in the right partition
    int objectCountRB = bins[rightIdx].nPrimitives;

    /* Determine the right part of c(L,R) = c_t + 1/SA(N) *(|N_L| * SA(N_L) + |N_R| * SA(N_R))
       Without dividing by SA(N) to reduce the number of computations
       That is, |N_L| * SA(N_L) + |N_R| * SA(N_R) */
    float cost = objectCountLB * surfaceArea(&bins[leftIdx]) + objectCountRB * surfaceArea(&bins[rightIdx]);
    
    if (cost < *bestCost)
    {
      *bestPos = testPos + (i + 1) * scale;
      *bestCost = cost;
    }
  }
  // After computing the min cost of the right part, the division by SA(N) plus c_t is applied
  *bestCost = 0.5 + *bestCost/surfaceArea(&s->bvh[nodeIdx]);
}

// Computete the bins AABB by determining where does a primitive's centroid lies
void buildBins(Scene* s, int nodeIdx, int axis, int nBins, AABB* bins)
{
  int nPrimitives = s->bvh[nodeIdx].nPrimitives;
  int first = s->bvh[nodeIdx].primitiveOffset;

  float minB = s->bvh[nodeIdx].minB[axis];
  float maxB = s->bvh[nodeIdx].maxB[axis];
  
  float extent = maxB - minB; 
  float binSize = extent/nBins;
  
  for(int i = 0; i < nPrimitives; i++)
  {
    PrimitiveInfo object = s->primitiveIDs[first+i];

    // Sphere Primitive
    if (object.type == SPHERE)
    {
      float sphereCenter = s->spheres[object.idx].center[axis];
      float sphereOffset = sphereCenter - minB;
  
      // printf("Sphere idx:%d, center: %f, offset: %f\n", sphereIdx, sphereCenter, sphereOffset);
      vec3 sphereMaxB, sphereMinB;
      glm_vec3_subs(s->spheres[object.idx].center, s->spheres[object.idx].radius, sphereMinB);
      glm_vec3_adds(s->spheres[object.idx].center, s->spheres[object.idx].radius, sphereMaxB);
  
      int binIdx = max(0, min(nBins-1, (int)sphereOffset/binSize));
      // printf("Bin index: %d\n", binIdx);
      glm_vec3_minv(bins[binIdx].minB, sphereMinB, bins[binIdx].minB);     
      glm_vec3_maxv(bins[binIdx].maxB, sphereMaxB, bins[binIdx].maxB);
      bins[binIdx].nPrimitives ++;        
      continue;
    }

    // Triangle Primitive
    float offset = s->triangles[object.idx].centroid[axis] - minB;
    int binIdx = max(0, min(nBins-1, (int)offset/binSize));

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

// Compute all possible partitions along an axis based on the number of bins
void computePossibleBinPartitions(AABB* bins, int nBins)
{
  for(int i = 0; i < nBins; i++)
  {
    // Expand left AABB of new possible partition
    int leftIdx = i + nBins;
  
    glm_vec3_minv(bins[nBins*3].minB, bins[i].minB, bins[nBins*3].minB);
    bins[nBins*3].nPrimitives += bins[i].nPrimitives;
    glm_vec3_maxv(bins[nBins*3].maxB, bins[i].maxB, bins[nBins*3].maxB);

    bins[leftIdx] = bins[nBins*3];

    // Expand left AABB of new possible partition
    int rightIdx = nBins - 1 - i + nBins*2;
    int binIdx = nBins - 1 - i;
    glm_vec3_minv(bins[nBins*3+1].minB, bins[binIdx].minB, bins[nBins*3+1].minB);
    bins[nBins*3+1].nPrimitives += bins[binIdx].nPrimitives;
    glm_vec3_maxv(bins[nBins*3+1].maxB, bins[binIdx].maxB, bins[nBins*3+1].maxB);

    bins[rightIdx] = bins[nBins*3+1];
  }
}

// Split the AABB of the subtree node in its left and right child node
int splitAABB(Scene* s, int parentIdx, int axis, int nodesUsed, float splitPos, AABB* bins, int nBins)
{
  int nPrimitives = s->bvh[parentIdx].nPrimitives;
  int primitiveOffset = s->bvh[parentIdx].primitiveOffset;

  // Order primitive IDs depending to which side of the partition they were assigned
  int i = primitiveOffset;
  int j = i + nPrimitives - 1;
  PrimitiveInfo temp;
  while (i <= j)
  {
    if (s->primitiveIDs[i].type == SPHERE && s->spheres[s->primitiveIDs[i].idx].center[axis] < splitPos)  i++;
    else if (s->primitiveIDs[i].type == TRIANGLE && s->triangles[s->primitiveIDs[i].idx].centroid[axis] < splitPos) i++;
    else {
      temp = s->primitiveIDs[i];
      s->primitiveIDs[i] = s->primitiveIDs[j];
      s->primitiveIDs[j] = temp;
      j--;
    }
  }

  int leftCount = i - primitiveOffset;
  if (leftCount == 0 || leftCount == nPrimitives) return nodesUsed;

  // Create left child with the primitives assigned to the first partition
  int leftChildIdx = nodesUsed ++;
  s->bvh[leftChildIdx].primitiveOffset = primitiveOffset;
  s->bvh[leftChildIdx].nPrimitives = leftCount;
  s->bvh[parentIdx].primitiveOffset = leftChildIdx;

  // Create right child with the primitives assigned to the second partition
  int rightChildIdx = nodesUsed ++ ;
  s->bvh[rightChildIdx].primitiveOffset = i;
  s->bvh[rightChildIdx].nPrimitives = nPrimitives - leftCount;

  // Repeat the process by partitioning the left and right child nodes
  updateBounds(s, leftChildIdx);
  nodesUsed = subdivide(s, leftChildIdx, nodesUsed, bins, nBins);
  updateBounds(s, rightChildIdx);
  nodesUsed = subdivide(s, rightChildIdx, nodesUsed, bins, nBins);

  /* Set the subtree root node's number of primtives to zero as only
     leaf nodes contain primitives */
  s->bvh[parentIdx].nPrimitives = 0;
  return nodesUsed;
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                  Model Loading
// ----------------------------------------------

// Mapping file content to memory using Linux libraries
#if defined(IS_LINUX)

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

// Mapping file content to memory using windows libraries
#elif defined(IS_WINDOWS)

#include <windows.h>
static char* mmap_file(size_t* len, const char* filename) {
    HANDLE hFile = CreateFile(
        filename,
        GENERIC_READ,
        FILE_SHARE_READ,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    if (hFile == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Could not open file %s (error %lu)\n", filename, GetLastError());
        return NULL;
    }

    LARGE_INTEGER fileSize;
    if (!GetFileSizeEx(hFile, &fileSize)) {
        fprintf(stderr, "Could not get file size (error %lu)\n", GetLastError());
        CloseHandle(hFile);
        return NULL;
    }

    HANDLE hMapping = CreateFileMapping(
        hFile,
        NULL,
        PAGE_READONLY,
        0,
        0,
        NULL
    );

    if (hMapping == NULL) {
        fprintf(stderr, "Could not create file mapping object (error %lu)\n", GetLastError());
        CloseHandle(hFile);
        return NULL;
    }

    char* p = (char*)MapViewOfFile(
        hMapping,
        FILE_MAP_READ,
        0,
        0,
        0
    );

    if (p == NULL) {
        fprintf(stderr, "Could not map view of file (error %lu)\n", GetLastError());
        CloseHandle(hMapping);
        CloseHandle(hFile);
        return NULL;
    }

    // Close the file handle and mapping handle; the mapped view will remain valid
    CloseHandle(hMapping);
    CloseHandle(hFile);

    *len = (size_t)fileSize.QuadPart;

    return p;
}
#endif

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
  // printf("Num Triangles: %d\n", s->nObjs[1]);

  int v0Idx, v1Idx, v2Idx;

  // printf("%ld\n", s->nMats);
  s->materials = genMaterialBuffer(s->nMats);
  for (size_t m = 0; m < s->nMats; m++) {
    s->materials[m] = material(s->mats[m]);
    // printf("%f %f %f\n", s->materials[m].albedo[0],s->materials[m].albedo[1],s->materials[m].albedo[2]);
    // printf("%f %f %f\n", s->materials[m].specular[0],s->materials[m].specular[1],s->materials[m].specular[2]);
    // printf("%f %f %f\n", s->materials[m].emission[0],s->materials[m].emission[1],s->materials[m].emission[2]);
    // printf("%f %f \n", s->materials[m].roughness,s->materials[m].metalness);
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