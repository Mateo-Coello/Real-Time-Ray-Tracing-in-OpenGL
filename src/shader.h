#ifndef SHADER_H
#define SHADER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <cglm/cglm.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "scene.h"

typedef struct {
  unsigned int ID;
} Shader;

Shader newComputeShader(const char* computeShaderPath);

Shader newShader(const char* vertexPath, const char* fragmentPath);

void useShader(Shader* s);

void setBool(Shader* s, const char * name, bool value);

void setInt(Shader* s, const char * name, int value);

void setUint(Shader* s, const char * name, int value);

void setFloat(Shader* s, const char * name, float value);

void setVec2(Shader* s, const char * name, vec2 value);

void setVec3(Shader* s, const char * name, vec3 value);

void setIVec4(Shader* s, const char* name, ivec4 value);

void setMat4(Shader* s, const char * name, mat4 value);

void genTextureBuffer(unsigned int* textureID, GLenum textureIdx,int binding, int width, int height);

void updateTextureBuffer(unsigned int textureID, GLenum textureIdx,int binding, int width, int height);

void genSceneSSBO(int nBuffers, unsigned int* sceneSSBO, Scene* s, Shader* program, int* bindingPoints);

void bindSceneAssetBuffers(unsigned int* sceneSSBO, int* bindingPoints);

#endif

