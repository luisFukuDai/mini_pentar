/***********************************************************************/
/*                                                                     */
/*  FILE        :global.h                                              */
/*  DATE        :Tue, Nov 19, 2013                                     */
/*  DESCRIPTION :Header file of globa.c                                */
/*  CPU TYPE    :RX621                                                 */
/*                                                                     */
/*  This file is generated by Shuhei KONDO.                            */
/*                                                                     */
/***********************************************************************/

#ifndef __GLOBAL__

#define __GLOBAL__

#include "queue.h"
#include "control.h"


extern int rxSci0;
extern int txSci0;
extern char buff[80];

extern char sprintfBuff[80];

extern MOTOR wheelMotor[4];

extern short sendSerialFlag;
extern short newData;

extern short r0M[2]; 
extern short r1M[2]; 

extern short r0S[2]; 
extern short r1S[2]; 

extern int servoData[2];  

#define dt 0.001

#endif