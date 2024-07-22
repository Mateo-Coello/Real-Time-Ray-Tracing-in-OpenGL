#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "cglm/cglm.h"
#include "shader.h"
#include "camera.h"
#include "renderInfo.h"
 
// Callbacks
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xPos, double yPos);
void scroll_callback(GLFWwindow* window, double xOffset, double yOffset);
void blockCursorCallback(GLFWwindow* window, int key, int scancode, int action, int mods); 
void processInput(GLFWwindow *window);
void sendCameraParameters(Shader* program, Camera* camera);
void renderTriangle();
void parseFloat3(const char* tripletInput, vec3 triplet);

// Window settings
unsigned int SCREEN_WIDTH = 1000;
unsigned int SCREEN_HEIGHT = 1000;
int blockCursor = 0;

// Camera Settings;
Camera camera;
float lastX, lastY;
float firstMouse = true;
mat4 pView, pProj; // previous inverse view and projection matrices

// Textures to render the ray traced frame
unsigned int currentFrameBuffer, hitBuffer, previousFrameBuffer, accumulationBuffer;

// Shader IDs
Shader screenShaderProgram;
Shader rayTracingShaderProgram;
Shader taaShaderProgram;

// Timing
RenderInfo info;
bool updateInfo = false;

int main(int argc, char* argv[])
{ 
  // glfw: initialize and configure
  // ------------------------------
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // glfw window creation
  // --------------------
  GLFWwindow* window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Ray Tracing", NULL, NULL);
  if (window == NULL)
  {
    printf("Failed to create GLFW window\n");
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(0);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSetCursorPosCallback(window, mouse_callback);
  glfwSetKeyCallback(window, blockCursorCallback);
  // glfwSetScrollCallback(window, scroll_callback);

  // glad: load all OpenGL function pointers
  // ---------------------------------------
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
  {
    printf("Failed to initialize GLAD\n");
    return -1;
  }

  const char* vendor = (const char*)glGetString(GL_VENDOR);
  const char* renderer = (const char*)glGetString(GL_RENDERER);

  printf("OpenGL Vendor: %s\n", vendor);
  printf("OpenGL Renderer: %s\n", renderer);
  
  // build and compile the screen shader and compute shader
  // ------------------------------------
  screenShaderProgram = newShader("shaders/vShader.glsl", "shaders/fShader.glsl");
  rayTracingShaderProgram = newComputeShader("shaders/RT_PBR.glsl");

  useShader(&screenShaderProgram);
  setInt(&screenShaderProgram, "currentFrameBuffer", 0);

  // Create image texture buffers to which render the ray traced image
  genTextureBuffer(&currentFrameBuffer, GL_TEXTURE0, 0, SCREEN_WIDTH, SCREEN_HEIGHT); // Current Frame Buffer
  genTextureBuffer(&hitBuffer, GL_TEXTURE1, 1, SCREEN_WIDTH, SCREEN_HEIGHT); // Hit Buffer
  genTextureBuffer(&accumulationBuffer, GL_TEXTURE2, 2, SCREEN_WIDTH, SCREEN_HEIGHT); // Accumulation Buffer
  genTextureBuffer(&previousFrameBuffer, GL_TEXTURE3, 3, SCREEN_WIDTH, SCREEN_HEIGHT); // Previous Frame Buffer
  
  // Initialize Camera and parameters related
  vec3 position = {0.0,0.5,2};
  initializeCamera(&camera, position, 90.0, (float)SCREEN_WIDTH, (float)SCREEN_HEIGHT);
  
  lastX = (float)SCREEN_WIDTH/2;
  lastY = (float)SCREEN_HEIGHT/2;

  if (argc < 2 || argc > 4) {
    printf("\n+--------------+\n"
           "| Instructions |\n"
           "+--------------+\n\n"
           "-Usage: ./raytracing \"(x,y,z)\" \"radius\" \"(r,g,b)\" (Optional) \n"
           "-Example: ./raytracing \"(0.0, 0.5, 0.0)\" \"0.5\" \n"
           "-Every component of the (r,g,b) triplet must be normalized [0.0,1.0].\n\n");
    return 1;
  }

  vec3 sphereCenter, color;

  parseFloat3(argv[1], sphereCenter);

  float sphereRadius = atof(argv[2]);

  // Output the parsed values for verification
  printf("Sphere center: (%f, %f, %f)\n", sphereCenter[0], sphereCenter[1], sphereCenter[2]);
  printf("Sphere radius: %f\n", sphereRadius);
  
  useShader(&rayTracingShaderProgram);
  setVec3(&rayTracingShaderProgram, "sphereCenter", sphereCenter);
  setFloat(&rayTracingShaderProgram, "sphereRadius", sphereRadius);

  if(argv[3]){
    parseFloat3(argv[3],color);
    setVec3(&rayTracingShaderProgram, "color", color);
    printf("Color: (%f, %f, %f)\n", color[0], color[1], color[2]);
  } else{
    glm_vec3_copy((vec3){-1,-1,-1},color);
    setVec3(&rayTracingShaderProgram, "color", color);
  }
  
  info = initRenderInfo();
  
  // render loop
  // -----------
  while (!glfwWindowShouldClose(window))
  { 
    // Framerate
    info.currentTime = (float)glfwGetTime();
    info.deltaTime = info.currentTime - info.lastTime;
    info.lastTime = info.currentTime;
    info.frameCount ++;
    info.totalFrames ++;
    
    // FPS
    char title[50];
    snprintf(title, sizeof(title), "Ray Tracing - Render Time: %.2fms", 1000*info.deltaTime);
    glfwSetWindowTitle(window, title);

    // input
    // -----
    processInput(window);
    if (updateInfo) 
    {
      resetRenderInfo(&info);
      updateInfo = false;
    }
    
    // Update current inverse view and projection matrices
    getInvViewProjectionMatrix(&camera);
    
    // Generate ray traced image
    // -------------------------
    useShader(&rayTracingShaderProgram);
    
    // The current frame (aka current time) is used as the seed for the generating random numbers   
    setFloat(&rayTracingShaderProgram, "uTime", info.currentTime);
    setInt(&rayTracingShaderProgram, "uFrameCount", info.frameCount);
    sendCameraParameters(&rayTracingShaderProgram, &camera);
    
    glDispatchCompute((unsigned int)(SCREEN_WIDTH/4), (unsigned int)(SCREEN_HEIGHT/8),1);
    
    // Update previous inverse view and projection matrices
    glm_mat4_copy(camera.view, pView);
    glm_mat4_copy(camera.proj, pProj);
    
    // Ray traced image
    useShader(&screenShaderProgram);
    renderTriangle();
  
    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    // -------------------------------------------------------------------------------
    glfwSwapBuffers(window);
    glfwPollEvents();
    
  }
  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  glDeleteTextures(1, &currentFrameBuffer);
  glDeleteTextures(1, &previousFrameBuffer);
  glDeleteTextures(1, &accumulationBuffer);
	glDeleteProgram(screenShaderProgram.ID);
	glDeleteProgram(rayTracingShaderProgram.ID);
  glDeleteProgram(taaShaderProgram.ID);

  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}

unsigned int triVAO = 0;
unsigned int triVBO;

void renderTriangle()
{
	if (triVAO == 0)
	{
		float triVertices[] = {
			// positions        // texture Coords
			-1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
			 3.0f, -1.0f, 0.0f, 3.0f, 0.0f,
			-1.0f,  3.0f, 0.0f, 0.0f, 3.0f,
		};
		// setup plane VAO
		glGenVertexArrays(1, &triVAO);
		glGenBuffers(1, &triVBO);
		glBindVertexArray(triVAO);
		glBindBuffer(GL_ARRAY_BUFFER, triVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(triVertices), &triVertices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	}
	glBindVertexArray(triVAO);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 3);
	glBindVertexArray(0);
}

void parseFloat3(const char* tripletInput, vec3 triplet)
{
  sscanf(tripletInput, "(%f,%f,%f)", &triplet[0], &triplet[1], &triplet[2]);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
  glViewport(0, 0, width, height);
  SCREEN_WIDTH = width;
  SCREEN_HEIGHT = height;
  info.frameCount = 1;
  
  camera.iWidth = width;
  camera.iHeight = height;

  updateTextureBuffer(currentFrameBuffer, GL_TEXTURE0, 0, width, height);
  updateTextureBuffer(hitBuffer, GL_TEXTURE1, 1, width, height);
  updateTextureBuffer(accumulationBuffer, GL_TEXTURE2, 2, width, height);
  updateTextureBuffer(previousFrameBuffer, GL_TEXTURE3, 3, width, height);
}

void mouse_callback(GLFWwindow* window, double xPosIn, double yPosIn){
  float xPos = (float)xPosIn;
  float yPos = (float)yPosIn;

  if (firstMouse)
  {
    lastX = xPos;
    lastY = yPos;
    firstMouse = false;
  }
  
  float xOffset = xPos - lastX;
  float yOffset = lastY - yPos;
  
  lastX = xPos;
  lastY = yPos;
  processMouseMovement(&camera, xOffset, yOffset, true);
  updateInfo = true;
}

void scroll_callback(GLFWwindow* window, double xOffset, double yOffset)
{
  processMouseScroll(&camera, (float)yOffset);
  updateInfo = true;
}

void processInput(GLFWwindow *window)
{
  if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);

  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    processKeyboard(&camera, FORWARD, &info);

  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    processKeyboard(&camera, BACKWARD, &info);
  
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    processKeyboard(&camera, LEFT, &info);
  
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    processKeyboard(&camera, RIGHT, &info);

  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
    processKeyboard(&camera, UP, &info);

  if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    processKeyboard(&camera, DOWN, &info);
}

void blockCursorCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
  if (key == GLFW_KEY_B && action == GLFW_PRESS) {
    blockCursor = !blockCursor ;
    glfwSetInputMode(window, GLFW_CURSOR, blockCursor ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
  }
}