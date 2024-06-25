// #include <cglm/vec3.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <memory.h>
// #include <stdbool.h>
// #include <string.h>

// #define TINYOBJ_LOADER_C_IMPLEMENTATION
// #include <tinyobj_loader_c.h>

// #include <float.h>
// #include <limits.h>
// #include <math.h>

// #include <fcntl.h>
// #include <sys/mman.h>
// #include <sys/stat.h>
// #include <sys/types.h>
// #include <unistd.h>
// #include <cglm/cglm.h>
// #include "hittable.h"
// #include "scene.h"

// static char* mmap_file(size_t* len, const char* filename) {
// #ifdef _WIN64
//   HANDLE file =
//       CreateFileA(filename, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING,
//                   FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, NULL);

//   if (file == INVALID_HANDLE_VALUE) { /* E.g. Model may not have materials. */
//     return NULL;
//   }

//   HANDLE fileMapping = CreateFileMapping(file, NULL, PAGE_READONLY, 0, 0, NULL);
//   assert(fileMapping != INVALID_HANDLE_VALUE);

//   LPVOID fileMapView = MapViewOfFile(fileMapping, FILE_MAP_READ, 0, 0, 0);
//   char* fileMapViewChar = (char*)fileMapView;
//   assert(fileMapView != NULL);

//   DWORD file_size = GetFileSize(file, NULL);
//   (*len) = (size_t)file_size;

//   return fileMapViewChar;
// #else

//   struct stat sb;
//   char* p;
//   int fd;

//   fd = open(filename, O_RDONLY);
//   if (fd == -1) {
//     perror("open");
//     return NULL;
//   }

//   if (fstat(fd, &sb) == -1) {
//     perror("fstat");
//     return NULL;
//   }

//   if (!S_ISREG(sb.st_mode)) {
//     fprintf(stderr, "%s is not a file\n", filename);
//     return NULL;
//   }

//   p = (char*)mmap(0, sb.st_size, PROT_READ, MAP_SHARED, fd, 0);

//   if (p == MAP_FAILED) {
//     perror("mmap");
//     return NULL;
//   }

//   if (close(fd) == -1) {
//     perror("close");
//     return NULL;
//   }

//   (*len) = sb.st_size;

//   return p;

// #endif
// }

// static void get_file_data(void* ctx, const char* filename, const int is_mtl,
//                           const char* obj_filename, char** data, size_t* len) {
//   // NOTE: If you allocate the buffer with malloc(),
//   // You can define your own memory management struct and pass it through `ctx`
//   // to store the pointer and free memories at clean up stage(when you quit an
//   // app)
//   // This example uses mmap(), so no free() required.
//   (void)ctx;

//   if (!filename) {
//     fprintf(stderr, "null filename\n");
//     (*data) = NULL;
//     (*len) = 0;
//     return;
//   }

//   size_t data_len = 0;

//   *data = mmap_file(&data_len, filename);
//   (*len) = data_len;
// }

// void loadModel(const char* filename, Scene s) 
// {  
//   unsigned int flags = TINYOBJ_FLAG_TRIANGULATE;
//   int ret =
//       tinyobj_parse_obj(&s.attrib, &s.shapes, &s.nShapes, &s.mats,
//                         &s.nMats, filename, get_file_data, NULL, flags);

//   if (ret != TINYOBJ_SUCCESS) {
//     printf("Error loading obj model\n");
//     return;
//   }
  
//   size_t nTri = s.attrib.num_face_num_verts;

//   int v0Idx, v1Idx, v2Idx;
  
//   for (size_t f = 0; f < s.attrib.num_faces; f+=3)
//   {
//     v0Idx = s.triangles[f/3].a = s.attrib.faces[ f ].v_idx;
//     v1Idx = s.triangles[f/3].b = s.attrib.faces[f+1].v_idx;
//     v2Idx = s.triangles[f/3].c = s.attrib.faces[f+2].v_idx;

//     vec3 v0 = {s.attrib.vertices[v0Idx*3], s.attrib.vertices[v0Idx*3+1], s.attrib.vertices[v0Idx*3+2]};
//     vec3 v1 = {s.attrib.vertices[v1Idx*3], s.attrib.vertices[v1Idx*3+1], s.attrib.vertices[v1Idx*3+2]}; 
//     vec3 v2 = {s.attrib.vertices[v2Idx*3], s.attrib.vertices[v2Idx*3+1], s.attrib.vertices[v2Idx*3+2]};

//     triangle(s.triangles[f/3], v0, v1, v2, 0);
    
//     printf("V%d:{%f,%f,%f} V%d:{%f,%f,%f} V%d:{%f,%f,%f}\n",
//            v0Idx, s.attrib.vertices[v0Idx*3], s.attrib.vertices[v0Idx*3+1], s.attrib.vertices[v0Idx*3+2], 
//            v1Idx, s.attrib.vertices[v1Idx*3], s.attrib.vertices[v1Idx*3+1], s.attrib.vertices[v1Idx*3+2], 
//            v2Idx, s.attrib.vertices[v2Idx*3], s.attrib.vertices[v2Idx*3+1], s.attrib.vertices[v2Idx*3+2]);
//   }
  
//   // TODO: Update the way triangles are handled by the RT shader
// }
