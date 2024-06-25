#include "src/random.h"

int main()
{
  int sqrtN = 1000;
  int insideCircle = 0;
  int insideCircleStratified = 0;
  
  unsigned int seed = genTimeSeed();
  
  for (int i=0; i < sqrtN; i++) 
  {
    for (int j=0; j < sqrtN; j++) {
      float x = randomFloat(&seed);
      float y = randomFloat(&seed);
    
      if (x*x + y*y < 1) insideCircle++;

      x = 2*((i+randomFloat(&seed))/sqrtN) - 1;
      y = 2*((j+randomFloat(&seed))/sqrtN) - 1;

      if (x*x + y*y < 1) insideCircleStratified++;
    }
  }

  printf("Estimate of PI: %f\n", (float)(4*insideCircle)/(sqrtN*sqrtN));
  printf("Stratified estimate of PI: %f\n", (float)(4*insideCircleStratified)/(sqrtN*sqrtN));
}
