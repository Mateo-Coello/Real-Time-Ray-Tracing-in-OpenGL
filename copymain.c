#include <cglm/mat4.h>
#include <cglm/types.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>
#include <cglm/cglm.h>
#include <cglm/vec3.h>
#include <cglm/affine.h>
#include "src/stb_image.h"
#include "src/shader.h"
#include "src/camera.h"

// Callbacks
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xPos, double yPos);
void scroll_callback(GLFWwindow* window, double xOffset, double yOffset);
void processInput(GLFWwindow *window);
void renderQuad();

// Window settings
unsigned int SCREEN_WIDTH = 1000;
unsigned int SCREEN_HEIGHT = 1000;

// Time struct
struct timeval currentTime;

int main()
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
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

  // glad: load all OpenGL function pointers
  // ---------------------------------------
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
  {
    printf("Failed to initialize GLAD\n");
    return -1;
  }
  
  // build and compile the screen shader and compute shader
  // ------------------------------------
  Shader screenShaderProgram = newShader("shaders/vShader.glsl", "shaders/fShader.glsl");
  Shader computeShaderProgram = newComputeShader("shaders/cShader.glsl");

  useShader(&screenShaderProgram);
  setInt(&screenShaderProgram, "screen", 0);

  // Texture
  unsigned int screenTex;
  glGenTextures(1, &screenTex);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, screenTex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);  

  int work_grp_cnt[3];
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &work_grp_cnt[0]);
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 1, &work_grp_cnt[1]);
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 2, &work_grp_cnt[2]);
	printf("Max work groups per compute shader\nx: %d\ny: %d\nz: %d\n",work_grp_cnt[0],work_grp_cnt[1],work_grp_cnt[2]);

	int work_grp_size[3];
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &work_grp_size[0]);
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 1, &work_grp_size[1]);
	glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 2, &work_grp_size[2]);
	printf("Max work groups per compute shader\nx: %d\ny: %d\nz: %d\n",work_grp_size[0],work_grp_size[1],work_grp_size[2]);

	int work_grp_inv;
	glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &work_grp_inv);
	printf("Max invocations count per work group: %d\n",work_grp_inv);
      
  // render loop
  // -----------
  while (!glfwWindowShouldClose(window))
  {
    // input
    // -----
    processInput(window);

    // Time seed to generate random numbers
    float timeSeed = (float)glfwGetTime();
    // gettimeofday(&currentTime, NULL);
    // vec2 timeSeed2 = {currentTime.tv_sec & 0xF0F0F0F0, currentTime.tv_usec };
    
    // render
    // ------
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, screenTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, SCREEN_WIDTH, SCREEN_HEIGHT, 0, GL_RGBA, GL_FLOAT, NULL);
    glBindImageTexture(0, screenTex, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);

    // Generate ray traced image
    useShader(&computeShaderProgram);
    setFloat(&computeShaderProgram, "timeSeed", timeSeed);
    glDispatchCompute((unsigned int)SCREEN_WIDTH/8, (unsigned int)SCREEN_HEIGHT/4, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      
    // Ray traced image
    useShader(&screenShaderProgram);
    renderQuad();
  
    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    // -------------------------------------------------------------------------------
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  glDeleteTextures(1, &screenTex);
	glDeleteProgram(screenShaderProgram.ID);
	glDeleteProgram(computeShaderProgram.ID);  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);
}

// renderQuad() renders a 1x1 XY quad in NDC
// -----------------------------------------
unsigned int quadVAO = 0;
unsigned int quadVBO;

void renderQuad()
{
	if (quadVAO == 0)
	{
		float quadVertices[] = {
			// positions        // texture Coords
			-1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
			-1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
			 1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
			 1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
		};
		// setup plane VAO
		glGenVertexArrays(1, &quadVAO);
		glGenBuffers(1, &quadVBO);
		glBindVertexArray(quadVAO);
		glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	}
	glBindVertexArray(quadVAO);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindVertexArray(0);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
  glViewport(0, 0, width, height);
  SCREEN_WIDTH = width;
  SCREEN_HEIGHT = height;
}

// void mouse_callback(GLFWwindow* window, double xPosIn, double yPosIn){
//   float xPos = (float)xPosIn;
//   float yPos = (float)yPosIn;

//   if (firstMouse)
//   {
//     lastX = xPos;
//     lastY = yPos;
//     firstMouse = false;
//   }
  
//   float xOffset = xPos - lastX;
//   float yOffset = lastY - yPos;
  
//   lastX = xPos;
//   lastY = yPos;
 
//   processMouseMovement(&camera, xOffset, yOffset, (GLboolean) true);
// }

// void scroll_callback(GLFWwindow* window, double xOffset, double yOffset)
// {
//   processMouseScroll(&camera, (float)yOffset);
// }

// void processInput(GLFWwindow *window)
// {
//   if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
//     glfwSetWindowShouldClose(window, true);

//   if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
//     processKeyboard(&camera, FORWARD, deltaTime);
  
//   if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
//     processKeyboard(&camera, BACKWARD, deltaTime);
    
//   if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
//     processKeyboard(&camera, LEFT, deltaTime);
    
//   if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
//     processKeyboard(&camera, RIGHT, deltaTime);

//   if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
//     processKeyboard(&camera, UP, deltaTime);
  
//   if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
//     processKeyboard(&camera, DOWN, deltaTime);
// }
