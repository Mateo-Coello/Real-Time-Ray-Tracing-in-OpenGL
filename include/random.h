#ifndef RANDOM_H
#define RANDOM_H

#include "cglm/cglm.h"
#include <sys/time.h>
#include <string.h>

unsigned int genTimeSeed();

float randomFloat(unsigned int* seed);

float randomFloatRange(unsigned int* seed, float min, float max);

void randomVec4(unsigned int* seed, vec4 dest);

void randomVec4Range(unsigned int* seed, float min, float max, vec4 dest);

#endif
