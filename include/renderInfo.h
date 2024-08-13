#ifndef RENDER_INFO_H
#define RENDER_INFO_H

#include "GLFW/glfw3.h"

typedef struct
{
  float currentTime;
  float lastTime;
  float deltaTime;
  float totalSeconds;
  // float fpsSum;
  int totalFrames;
  int frameCount;
} RenderInfo;

RenderInfo initRenderInfo();

void resetRenderInfo(RenderInfo* info);

#endif
