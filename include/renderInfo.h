#ifndef RENDER_INFO_H
#define RENDER_INFO_H

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
