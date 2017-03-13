#include "iodefine.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "interrupt_handlers.h"
#include "global.h"
#include "sci.h"
#include "adc.h"
#include "timer.h"
#include "pwm.h"
#include "encoder.h"
#include "queue.h"
#include "control.h"
#include "gyro.h"
#include "accel.h"

ACCEL SCAT100;

float adaptFilterAccel(float x)
{

  static float xHat;
  static float dXHat;
  static float w0 = 40.0;
  static int first;

  static float dqHat;

  float alpha;
  float e;

  if(first == 1) {
    first = 1;
    xHat = x;
  }

  e = x-xHat;

  xHat  = xHat + dt*(dXHat + 2*w0*e); 
  dXHat = dXHat + dt*( w0*w0*e); 


  alpha = fabs(dXHat) + 0.1;

  if(alpha >= 0.999)
    alpha = 0.999;

  dqHat = (1-alpha)*x + alpha*dqHat;


  return dqHat;
}


void initAccel(void)
{
  SCAT100.xAdChannel = 1;
  SCAT100.yAdChannel = 2;
  SCAT100.VOLTS_PER_G = 1.0;

}

float accelCalcAngle(ACCEL *a)
{

  a->xRaw = getAdVal(a->xAdChannel);
  a->yRaw = getAdVal(a->yAdChannel);


#ifdef ROBOT1
  float temp;

  temp = (a->xRaw-2048.0)/2048.0;  

  if (temp <= 1.0 && temp >= -1.0)//check for invalid values for asin
    a->q = asin(temp);

#else
  a->q = -atan2((a->xRaw-2048.0)/2048.0,(a->yRaw-2048.0)/2048.0);
#endif


 a->qF = adaptFilterAccel(a->q);


  return a->qF;
}



