#include "stdio.h"
#include "sci.h"
#include "sci1.h"
#include "digitalServo.h"

//static int fd;
static int data[32];

int servoId[32]={0x01,0x02,0x05};
int servoCount = 2;

void usleep(int i)
{
  i *= 10000;

  for(;i>0;i--)
    asm("nop");

} 

int setupServos()
{

  multiServoCommand(servoId,servoCount,REBOOT,data);  
  usleep(100);

  data[0] = ON;
  data[1] = ON;
  multiServoCommand(servoId,servoCount,TORQUE_ON_OFF,data);
  usleep(100);

  return 0;

}


int calcSum(char *c, int k)
{

  int i;
  int sum;

  sum = 0;
  for( i = 0 ; i < k ; i++) {
    sum ^= c[i];
  }

  return sum&0xFF;
}

int servoCommand(int servo, int cmd, int data)
{
  short i;
  char packet[80];

  char FLAG[]     = {0x20, 0x01, 0x01, 0x01};
  char ADDRESS[]  = {0xFF, 0x24, 0x1E, 0x06};
  char LEN[]      = {0x00, 0x01, 0x02, 0x01};
  char CNT[]      = {0x00, 0x01, 0x01, 0x01};

  /*packet header*/
  packet[0] = 0xFA;
  packet[1] = 0xAF;

  /*servo id*/
  packet[2] = (char) servo;

  /*flag values*/
  packet[3] = FLAG[cmd];

  /*address value*/
  packet[4] = ADDRESS[cmd];

  /*length of data*/
  packet[5] = LEN[cmd];

  /*number of servos*/
  packet[6] = CNT[cmd];

  for(i = 0 ; i < LEN[cmd] ; i++) {
    packet[i+7] = (data >> ((i)*8))&0xFF;
  }

  packet[i+7] = (char) calcSum(packet+2,i+7-2);

  sci1Write(packet,i+8);

  return 0;
}


int multiServoCommand(int *servo, int servoCount, int cmd, int *data)
{
  short i,j;
  short k;
  char packet[80];

//  char FLAG[]     = {0x20, 0x01, 0x01, 0x01};
  char ADDRESS[]  = {0xFF, 0x24, 0x1E, 0x06};
  char LEN[]      = {0x00, 0x02, 0x03, 0x03};

  /*packet header*/
  packet[0] = 0xFA;
  packet[1] = 0xAF;

  /*servo id*/
  packet[2] = 0;

  /*flag values*/
  packet[3] = 0;

  /*address value*/
  packet[4] = ADDRESS[cmd];

  /*length of data*/
  packet[5] = LEN[cmd];

  /*number of servos*/
  packet[6] = servoCount;

  for(j = 0,k = 7 ; j < servoCount ; j++) {

    packet[k] = (char) servo[j];

    for(i = 0 ; i < LEN[cmd] ; i++) {
      packet[i+k+1] = (data[j] >> ((i)*8))&0xFF;
    }
    k += i;
  }

  packet[k] = (char) calcSum(packet+2,k-2);

  sci1Write(packet,k+8+1);

  

  return 0;
}

int disableServos()
{

  data[0] = OFF;
  data[1] = OFF;
  multiServoCommand(servoId,servoCount,TORQUE_ON_OFF,data);

  return 0;
}


