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



static short packet[9];

int formPacket()
{
  static short n=-1;
  static short checksum;

  short i;
  short data;

  data = sci0Getc();//check for new serial data

  if(data == -1)// return if none
    return 0;

  if( (data&0xFF80) == 0x80){// if binary value is 0x80 then packet starts
    n = 0;
    checksum = 0;
  }

  if(n >= 0)
    packet[n++] = data&0xFF;// use only lower 7-bits

  if(n >= 9){// when data packet has 9 entries then packet is complete
    n = -1;

    for(i = 0 ; i < 8 ; i++) 
      checksum ^= (0xFF&packet[i]);

    if( (0x7F&checksum) == (0x7F&packet[8]) )
      return 1;
    else
      return 0;
  }

  return 0;
}

void decodePacket()
{

  r0M[0] = (packet[0]&0x7f);
  r0M[0] |= (packet[1]&0b00011111)*128;
  r0M[0] = r0M[0] - r0M[0]/0x800*4096;

  r0M[1] = (packet[1]&0b01100000)/32;
  r0M[1] |= (packet[2]&0b01111111)*4;
  r0M[1] |= (packet[3]&0b00000111)*512;
  r0M[1] = r0M[1] - r0M[1]/0x800*4096;

  r0S[0] = (packet[3]&0b00001000)/8;
  r0S[1] = (packet[3]&0b00010000)/16;

  r1M[0] = (packet[4]&0x7f);
  r1M[0] |= (packet[5]&0b00011111)*128;
  r1M[0] = r1M[0] - r1M[0]/0x800*4096;

  r1M[1] = (packet[5]&0b01100000)/32;
  r1M[1] |= (packet[6]&0b01111111)*4;
  r1M[1] |= (packet[7]&0b00000111)*512;
  r1M[1] = r1M[1] - r1M[1]/0x800*4096;

  r1S[0] = (packet[7]&0b00001000)/8;
  r1S[1] = (packet[7]&0b00010000)/16;


  sprintf(sprintfBuff,"%d %d %d %d\n",r0M[0],r0M[1],r0S[0],r0S[1]);
  sci0Puts(sprintfBuff);

  sprintf(sprintfBuff,"%d %d %d %d\n",r1M[0],r1M[1],r1S[0],r1S[1]);
  sci0Puts(sprintfBuff);

}  

int checkPacket(void)
{

  if (formPacket() == 1){
    newData = 1;
    decodePacket();
    sci0Puts("decoded");
    sci0Putc(newData+0x30);
    sci0Puts("\n");

#ifdef ROBOT0
    servoData[0] = r0S[0] != 0 ? 1500:-1500;
    servoData[1] = r0S[1] != 0 ? 1500:-1500;
#else
    servoData[0] = r1S[0] != 0 ? 1500:-1500;
    servoData[1] = r1S[1] != 0 ? 1500:-1500;
#endif

    multiServoCommand(servoId,servoCount,SET_ANGLE,servoData); 
  }

  return 0;
}
