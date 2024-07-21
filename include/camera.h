#ifndef CAMERA_H
#define CAMERA_H

#include "glad/glad.h"
#include "cglm/cglm.h"
#include "cglm/vec3.h"
#include "cglm/affine.h"
#include <math.h>
#include "shader.h"
#include "renderInfo.h"

#define YAW         -90.0f;
#define PITCH        0.0f;
#define SPEED        2.5f;
#define SENSITIVITY  0.1f;
#define ZOOM         45.0f;
#define FAR_PLANE    1000.0;
#define NEAR_PLANE   0.01;

typedef enum {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  UP,
  DOWN
}Camera_Movement;

typedef struct {
  vec3 position, front, worldUp, right, up;
  float iWidth, iHeight, vfov, pitch, yaw, movementSpeed, mouseSensitivity;
  mat4 proj, view, invProj, invView;
  mat4 rotMat, transMat, invT;
} Camera;

void initializeCamera(Camera* camera, vec3 position, float vfov, float iWidth, float iHeight);

void getViewMatrix(Camera* camera);

void getProjectionMatrix(Camera* camera);

void getInvViewProjectionMatrix(Camera* camera);

void processKeyboard(Camera* camera, Camera_Movement direction, RenderInfo* info);

void processMouseMovement(Camera* camera, float xOffset, float yOffset, bool constraintPitch);

void processMouseScroll(Camera* instance, float yOffset);

void updateCameraVectors(Camera *camera);

#endif
