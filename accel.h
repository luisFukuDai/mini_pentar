#ifndef __ACCEL__
#define __ACCEL__

typedef struct accel{
  float VOLTS_PER_G;
  int xAdChannel;
  int yAdChannel;
  int xRaw;
  int yRaw;
  float q;
  float qF;
}ACCEL;

void initAccel(void);
float accelCalcAngle(ACCEL *a); 
#endif
