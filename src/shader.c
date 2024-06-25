#include "shader.h"

void checkCompileErrors(unsigned int shader, const char * type);

void readFile(const char* filePath, char** content);

#define MORE_CHARS 1024

Shader newComputeShader(const char* computeShaderPath)
{
  char* computeShaderCode;

  readFile(computeShaderPath, &computeShaderCode);
  const char * computeSC = computeShaderCode;
  unsigned int compute;
 
  compute = glCreateShader(GL_COMPUTE_SHADER);
  glShaderSource(compute, 1, &computeSC, NULL);
  glCompileShader(compute);
  checkCompileErrors(compute, "COMPUTE");

  Shader s;
  s.ID = glCreateProgram();
  glAttachShader(s.ID, compute);
  glLinkProgram(s.ID);
  checkCompileErrors(s.ID, "PROGRAM");

  glDeleteShader(compute);
  
  return s;
}

Shader newShader(const char* vertexPath, const char* fragmentPath)
{
  char * vertexCode;
  char * fragmentCode;

  readFile(vertexPath, &vertexCode);
  readFile(fragmentPath, &fragmentCode);

  const char * vShaderCode = vertexCode;
  const char * fShaderCode = fragmentCode;

  unsigned int vertex, fragment;
  
  vertex = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex, 1, &vShaderCode, NULL);
  glCompileShader(vertex);
  checkCompileErrors(vertex, "VERTEX");
  
  fragment = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment, 1, &fShaderCode, NULL);
  glCompileShader(fragment);
  checkCompileErrors(fragment, "FRAGMENT");

  Shader s;
  s.ID = glCreateProgram();
  glAttachShader(s.ID, vertex);
  glAttachShader(s.ID, fragment);
  glLinkProgram(s.ID);
  checkCompileErrors(s.ID, "PROGRAM");

  glDeleteShader(vertex);
  glDeleteShader(fragment);
  
  return s;
}

void useShader(Shader* s){
  glUseProgram(s->ID);
}

void setBool(Shader* s, const char * name, bool value){
  glUniform1i(glGetUniformLocation(s->ID, name), (int) value);
}

void setInt(Shader* s, const char * name, int value){
  glUniform1i(glGetUniformLocation(s->ID, name), value);
}

void setUint(Shader* s, const char * name, int value){
  glUniform1ui(glGetUniformLocation(s->ID, name), value);
}

void setFloat(Shader* s, const char * name, float value){
  glUniform1f(glGetUniformLocation(s->ID, name), value);
}

void setVec2(Shader* s, const char* name, vec2 value){
  glUniform2fv(glGetUniformLocation(s->ID, name), 1, value);
}

void setVec3(Shader* s, const char* name, vec3 value){
  glUniform3fv(glGetUniformLocation(s->ID, name), 1, value);
}

void setIVec4(Shader* s, const char* name, ivec4 value){
  glUniform4iv(glGetUniformLocation(s->ID, name), 1, value);
}

void setMat4(Shader* s, const char * name, mat4 value){
  glUniformMatrix4fv(glGetUniformLocation(s->ID, name), 1, GL_FALSE, value[0]);
}

// Modify the readFile function to accept a double pointer (char**) for content
void readFile(const char* filePath, char** content) {
  FILE* file = fopen(filePath, "r");

  if (file == NULL) {
    printf("Error opening file.\n");
    return;
  }

  // Initialize the total_chars and allocate initial memory
  size_t total_chars = 0;
  char c;
  *content = NULL;

  do {
    c = fgetc(file);

    if (ferror(file)) {
      printf("Error reading from file.\n");
      return;
    }

    if (feof(file)) {
      if (total_chars != 0) {
        (*content) = realloc((*content), total_chars + 1);
        (*content)[total_chars] = '\0';
      }
      break;
    }

    // Allocate memory as needed
    if (total_chars == 0) {
      (*content) = malloc(MORE_CHARS);
    }

    (*content)[total_chars] = c;
    total_chars++;

    if (total_chars % MORE_CHARS == 0) {
      size_t new_size = total_chars + MORE_CHARS;
      (*content) = realloc((*content), new_size);
    }

  } while (true);

  // printf("The content of %s is the following:\n%s\n", filePath, (*content));
}

void checkCompileErrors(unsigned int shader, const char * type){
  int success;
  char infoLog[1024];
  if (strcmp(type, "PROGRAM")){
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success){
      glGetShaderInfoLog(shader, 1024, NULL, infoLog);
      printf("ERROR: %s_COMPILATION_ERROR\n%s\n", type, infoLog);
    }
  } 
  else {
    glGetProgramiv(shader, GL_LINK_STATUS, &success);
    if (!success){
      glGetProgramInfoLog(shader, 1024, NULL, infoLog);
      printf("ERROR: PROGRAM_LINKING_ERROR\n%s\n", infoLog);
    }
  }
}

void genTextureBuffer(unsigned int* textureID, GLenum textureIdx, int binding, int width, int height)
{
  glGenTextures(1, textureID);
  glActiveTexture(textureIdx);
  glBindTexture(GL_TEXTURE_2D, *textureID);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
  glBindImageTexture(binding, *textureID, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
}

void updateTextureBuffer(unsigned int textureID, GLenum textureIdx, int binding, int width, int height)
{
  glActiveTexture(textureIdx);
	glBindTexture(GL_TEXTURE_2D, textureID);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
  glBindImageTexture(binding, textureID, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
}

void genSceneSSBO(int nBuffers, unsigned int* sceneSSBO, Scene* s, Shader* program, int* bindingPoints)
{
  glGenBuffers(nBuffers, sceneSSBO);

  unsigned int spheresBlockIdx = glGetProgramResourceIndex(program->ID, GL_SHADER_STORAGE_BLOCK, "SpheresData");
  unsigned int verticesBlockIdx = glGetProgramResourceIndex(program->ID, GL_SHADER_STORAGE_BLOCK, "VerticesData");
  unsigned int trianglesBlockIdx = glGetProgramResourceIndex(program->ID, GL_SHADER_STORAGE_BLOCK, "TrianglesData");
  unsigned int materialsBlockIdx = glGetProgramResourceIndex(program->ID, GL_SHADER_STORAGE_BLOCK, "MaterialsData");
  unsigned int objectIDsBlockIdx = glGetProgramResourceIndex(program->ID, GL_SHADER_STORAGE_BLOCK, "ObjectIDs");
  unsigned int objectsBVHBlockIdx = glGetProgramResourceIndex(program->ID, GL_SHADER_STORAGE_BLOCK, "ObjectsBVH");
  // unsigned int lightsBlockIdx = glGetProgramResourceIndex(program->ID, GL_SHADER_STORAGE_BLOCK, "LightsData");
  GLenum property = GL_BUFFER_BINDING;
  glGetProgramResourceiv(program->ID, GL_SHADER_STORAGE_BLOCK, spheresBlockIdx, 1, &property, 1, NULL, &bindingPoints[0]);
  glGetProgramResourceiv(program->ID, GL_SHADER_STORAGE_BLOCK, verticesBlockIdx, 1, &property, 1, NULL, &bindingPoints[1]);
  glGetProgramResourceiv(program->ID, GL_SHADER_STORAGE_BLOCK, trianglesBlockIdx, 1, &property, 1, NULL, &bindingPoints[2]);
  glGetProgramResourceiv(program->ID, GL_SHADER_STORAGE_BLOCK, materialsBlockIdx, 1, &property, 1, NULL, &bindingPoints[3]);
  glGetProgramResourceiv(program->ID, GL_SHADER_STORAGE_BLOCK, objectIDsBlockIdx, 1, &property, 1, NULL, &bindingPoints[4]);
  glGetProgramResourceiv(program->ID, GL_SHADER_STORAGE_BLOCK, objectsBVHBlockIdx, 1, &property, 1, NULL, &bindingPoints[5]);
  // glGetProgramResourceiv(program->ID, GL_SHADER_STORAGE_BLOCK, lightsBlockIdx, 1, &property, 1, NULL, &bindingPoints[6]);
  
  // Spheres
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, sceneSSBO[0]);
  glBufferStorage(GL_SHADER_STORAGE_BUFFER, s->nObjs[0] * sizeof(Sphere), s->spheres, GL_DYNAMIC_STORAGE_BIT);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[0], sceneSSBO[0]);

  // Triangle Vertices
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, sceneSSBO[1]);
  glBufferStorage(GL_SHADER_STORAGE_BUFFER, s->attrib.num_vertices * 3 * sizeof(float), s->attrib.vertices, GL_DYNAMIC_STORAGE_BIT);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[1], sceneSSBO[1]);
  
  // Triangles
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, sceneSSBO[2]);
  glBufferStorage(GL_SHADER_STORAGE_BUFFER, s->nObjs[1] * sizeof(Triangle), s->triangles, GL_DYNAMIC_STORAGE_BIT);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[2], sceneSSBO[2]);

  // Materials
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, sceneSSBO[3]);
  glBufferStorage(GL_SHADER_STORAGE_BUFFER, s->nMats * sizeof(Material), s->materials, GL_DYNAMIC_STORAGE_BIT);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[3], sceneSSBO[3]);
  
  // Object IDs
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, sceneSSBO[4]);
  glBufferStorage(GL_SHADER_STORAGE_BUFFER, s->nObjs[2] * sizeof(PrimitiveInfo), s->objectIDs, GL_DYNAMIC_STORAGE_BIT);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[4], sceneSSBO[4]);

  // Objects BVH
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, sceneSSBO[5]);
  glBufferStorage(GL_SHADER_STORAGE_BUFFER, s->nodesUsed * sizeof(Node), s->bvh, GL_DYNAMIC_STORAGE_BIT);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[5], sceneSSBO[5]);
  
  // Lights
  // glBindBuffer(GL_SHADER_STORAGE_BUFFER, sceneSSBO[6]);
  // glBufferStorage(GL_SHADER_STORAGE_BUFFER, s->nObjs[3] * sizeof(Sphere), s->lights, GL_DYNAMIC_STORAGE_BIT);
  // glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[6], sceneSSBO[6]);
}

void bindSceneAssetBuffers(unsigned int* sceneSSBO, int* bindingPoints)
{
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[0], sceneSSBO[0]);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[1], sceneSSBO[1]);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[2], sceneSSBO[2]);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[3], sceneSSBO[3]);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[4], sceneSSBO[4]);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[5], sceneSSBO[5]);
  // glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoints[6], sceneSSBO[6]);
}