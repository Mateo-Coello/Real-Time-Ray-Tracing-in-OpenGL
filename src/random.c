#include "random.h"

unsigned int genTimeSeed()
{
  struct timeval currentTime;
  gettimeofday(&currentTime, NULL);
  unsigned int seed = currentTime.tv_sec%1000*currentTime.tv_usec;
  printf("%u\n",seed);
  return seed;
}

unsigned int pcgHash(unsigned int seed)
{
    unsigned int state = seed * 747796405U + 2891336453U;
    unsigned int word = ((state >> ((state >> 28U) + 4U)) ^ state) * 277803737U;
    return (word >> 22U) ^ word;
}

// Generates random number between [0,1)
float randomFloat(unsigned int* seed)
{
    *seed = pcgHash(*seed);
    return *seed/(float)0xffffffffU;
}

// Generates random number between [min, max)
float randomFloatRange(unsigned int* seed, float min, float max)
{
    return min + (max-min)*randomFloat(seed);
}

// Generates random vector whose components are between [0,1)
void randomVec4(unsigned int* seed, vec4 dest)
{
  float x, y, z;
  x = randomFloat(seed);
  y = randomFloat(seed);
  z = randomFloat(seed);
  glm_vec4((vec3){x,y,z}, 0.0, dest);
}

// Generates random vector whose components are between [min,max)
void randomVec4Range(unsigned int* seed, float min, float max, vec4 dest)
{
  float x, y, z;
  x = randomFloatRange(seed, min, max);
  y = randomFloatRange(seed, min, max);
  z = randomFloatRange(seed, min, max);
  glm_vec4((vec3){x,y,z}, 0.0, dest);
}


