#include "camera.h"
#include "renderInfo.h"
#include "cglm/affine-pre.h"
#include "cglm/affine.h"
#include "cglm/mat4.h"
#include "cglm/simd/sse2/mat4.h"

void initializeCamera(Camera* camera, vec3 position, float vfov, float iWidth, float iHeight){
  glm_vec3_copy(position, camera->position);
  glm_vec3_copy((vec3){0.0, 1.0, 0.0}, camera->worldUp);
  camera->iWidth = iWidth;
  camera->iHeight = iHeight;
  camera->yaw = YAW;
  camera->pitch = PITCH;
  camera->movementSpeed = SPEED;
  camera->mouseSensitivity = SENSITIVITY;
  camera->vfov = vfov;
  updateCameraVectors(camera);
}

void getViewMatrix(Camera* camera)
{
  vec3 target;
  glm_vec3_add(camera->position, camera->front, target);

  glm_lookat(camera->position, target, camera->up, camera->view);

  glm_mat4_inv_sse2(camera->view, camera->invView);
}

void getProjectionMatrix(Camera* camera)
{
  glm_perspective(glm_rad(camera->vfov), camera->iWidth/camera->iHeight, 0.01, 9999999.0, camera->proj);
  glm_mat4_inv_sse2(camera->proj, camera->invProj);
}

void getInvViewProjectionMatrix(Camera* camera)
{
  vec3 target;
  glm_vec3_add(camera->position, camera->front, target);
  glm_lookat(camera->position, target, camera->up, camera->view);
  
  glm_perspective(glm_rad(camera->vfov), camera->iWidth/camera->iHeight, 0.01, 9999999.0, camera->proj);
 
  glm_mat4_inv_sse2(camera->view, camera->invView);
  glm_mat4_inv_sse2(camera->proj, camera->invProj);
}

void getInvTransformationMatrix(Camera* camera)
{
  mat4 t;
  glm_mat4_mul(camera->rotMat, camera->transMat, t);
  glm_mat4_inv_sse2(t, camera->invT);
}

void processKeyboard(Camera* camera, Camera_Movement direction, RenderInfo* info)
{    
  resetRenderInfo(info);
  
  float velocity = camera->movementSpeed * info->deltaTime;
  
  if (direction == FORWARD){
    vec3 shift;
    glm_vec3_scale(camera->front, velocity, shift); 
    glm_translate_make(camera->transMat, shift);
    glm_vec3_add(camera->position, shift, camera->position);
    
  }
  if (direction == BACKWARD){
    vec3 shift;
    glm_vec3_scale(camera->front, velocity, shift);
    glm_translate_make(camera->transMat, shift);
    glm_vec3_sub(camera->position, shift, camera->position);
  }
  if (direction == RIGHT){
    vec3 shift;
    glm_vec3_scale(camera->right, velocity, shift);
    glm_translate_make(camera->transMat, shift);
    glm_vec3_add(camera->position, shift, camera->position);
  }
  if (direction == LEFT){
    vec3 shift;
    glm_vec3_scale(camera->right, velocity, shift); 
    glm_translate_make(camera->transMat, shift);
    glm_vec3_sub(camera->position, shift, camera->position);
  }
  if (direction == UP){
    vec3 shift = {0.0f, velocity, 0.0f};
    glm_translate_make(camera->transMat, shift);
    glm_vec3_add(camera->position, shift, camera->position); 
  }
  if (direction == DOWN){
    vec3 shift = {0.0f, -velocity, 0.0f};
    glm_translate_make(camera->transMat, shift);
    glm_vec3_add(camera->position, shift, camera->position); 
  }
}

void processMouseMovement(Camera* camera, float xOffset, float yOffset, bool constraintPitch)
{  
  xOffset *= camera->mouseSensitivity;
  yOffset *= camera->mouseSensitivity;

  camera->yaw   += xOffset;
  camera->pitch += yOffset;

  if (constraintPitch){
    if (camera->pitch > 89.0f)
      camera->pitch = 89.0f;
    if (camera->pitch < -89.0f)
      camera->pitch = -89.0f;
  }
  updateCameraVectors(camera);

  float cosX = cos(glm_rad(xOffset));
  float cosY = cos(glm_rad(yOffset));
  float sinX = sin(glm_rad(xOffset));
  float sinY = sin(glm_rad(yOffset));
  glm_mat4_copy((mat4){{cosY, sinY*sinX, sinY*cosX, 0},
                       {0, cosX, -sinY, 0},
                       {-sinY, cosY*sinX, cosY*cosX, 0},
                       {0, 0, 0, 1}},camera->rotMat);
}

void processMouseScroll(Camera* camera, float yOffset){  
  camera->vfov -= yOffset;
  if (camera->vfov < 1.0f)
    camera->vfov = 1.0f;
  if (camera->vfov > 45.0f)
    camera->vfov = 45.0f;
} 

void updateCameraVectors(Camera *camera)
{
  vec3 front, target;
  mat4 view, projection;
  float viewportW, viewportH;
  
  // Calculate the lookAt direction vector
  front[0] = cos(glm_rad(camera->yaw)) * cos(glm_rad(camera->pitch));
  front[1] = sin(glm_rad(camera->pitch));
  front[2] = sin(glm_rad(camera->yaw)) * cos(glm_rad(camera->pitch));
  glm_normalize(front);
  glm_vec3_copy(front, camera->front);
  
  // Calculate the right and up  (u,v basis vectors) for the camera coordinate frame. 
  glm_vec3_cross(front, camera->worldUp, camera->right);
  glm_vec3_normalize(camera->right);
  glm_vec3_cross(camera->right, front, camera->up);
  glm_vec3_normalize(camera->up);
}

void sendCameraParameters(Shader* program, Camera* camera)
{
  setVec3(program, "position", camera->position);
  setMat4(program, "invView", camera->invView);
  setMat4(program, "invProj", camera->invProj);
  // setVec3(program, "front", camera->front);
  // setVec3(program, "right", camera->right);
  // setVec3(program, "up", camera->up);
}
