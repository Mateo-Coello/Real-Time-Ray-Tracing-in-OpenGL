#include "renderInfo.h"

RenderInfo initRenderInfo()
{
  RenderInfo info;
  info.currentTime= 0;
  info.lastTime= (float)glfwGetTime();
  info.deltaTime = 0;
  info.totalSeconds = 0;
  info.frameCount = 1;
  info.totalFrames = 0;
  
  return info;
}

void resetRenderInfo(RenderInfo* info)
{
  info->frameCount = 1;
}