/***********************************************************************/
/*                                                                     */
/*  FILE        :main.c                                                */
/*  DATE        :Tue, Nov 19, 2013                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :RX621                                                 */
/*                                                                     */
/*  This file is generated by Luis Canete.                             */
/*                                                                     */
/***********************************************************************/

#include "iodefine.h"
#include "stdio.h"
#include "math.h"
#include "interrupt_handlers.h"
#include "global.h"
#include "sci.h"
#include "sci1.h"
#include "adc.h"
#include "timer.h"
#include "pwm.h"
#include "encoder.h"
#include "queue.h"
#include "control.h"
#include "digitalServo.h"
#include "gyro.h"
#include "inverted.h"
#include "accel.h"
#include "packet.h"

extern GYRO CRS02;
extern INV_PEN R0;
extern ACCEL SCAT100;
extern float adaptDq;
extern float adaptDqTheta;

extern float ti;

int main(void)
{
  int i=0;

  startS12AD();
  startDcPWM(); 
  setupServos();

  startEnc1();
  startEnc2();

  initGyro();
  initAccel();

  initMotorParam();
  setMotorPwmValue(0,0.0);
  setMotorPwmValue(1,0.0);

  initInvVars();

  servoData[0] = -1500;
  servoData[1] = -1500;

  multiServoCommand(servoId,servoCount,SET_ANGLE,servoData); 

  //  setDuty(servoL,0.047);
  //  setDuty(servoS,0.078);


  while(1){

    checkPacket();
    if(R0.psi.q > 3.140 || R0.psi.q < -3.14)
      PORTA.DR.BIT.B0 = 0;

    if(sendSerialFlag >= 10) {
      i++;
      i%=100;
      sprintf(sprintfBuff,"%08ld  \
          \t%ld \
          \t%ld \
          \t%ld \
          \t%ld \
          \t%ld \
          \n",
          (long) (1000*ti), 
          (long) (R0.psi.q*1000), 
          (long) (R0.psi.dQ*1000), 
          (long) (R0.thetaW.q*1000), 
          (long) (R0.thetaW.dQ*1000), 
          (long) (R0.phi.q*1000) 
          );

      sci0Puts(sprintfBuff);

      sendSerialFlag = 0;
      if (i == 0)
        PORTA.DR.BIT.B0 = ~PORTA.DR.BIT.B0;

    }


  }

  return 0;
}

