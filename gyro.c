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

GYRO CRS02;

void initGyro(void)
{
  CRS02.adChannel = 0;
  CRS02.DEGREES_PER_SEC = 100.0;
  CRS02.OFFSET = 2048.0;
  CRS02.dQ = 0.0;
  CRS02.raw = 0;

}

float gyroCalcDQ(GYRO *g)
{
  g->raw = getAdVal(g->adChannel);
  g->dQ = (g->raw - g->OFFSET)/2048.0*g->DEGREES_PER_SEC/180.0*M_PI;

  return g->dQ;
}



