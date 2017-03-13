#ifndef __GYRO__
#define __GYRO__

typedef struct gyro{
  float DEGREES_PER_SEC;
  float OFFSET;
  float dQ;
  int adChannel;
  int raw;
}GYRO;

void initGyro(void);
float gyroCalcDQ(GYRO *g); 

#endif
