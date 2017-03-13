#ifndef __DIGI_SERVO__
#define __DIGI_SERVO__

#define REBOOT  0
#define TORQUE_ON_OFF 1
#define SET_ANGLE 0x02
#define SET_BAUD 0x03

#define ON 0x01
#define OFF 0x00
#define BREAK 0x02
#define BAUD_RATE 0x05

int calcSum(char *c, int k);
int servoCommand(int servo, int cmd, int data);
int multiServoCommand(int *servo, int servoCount, int cmd, int *data);
int setupServos();

extern int servoId[32];
extern int servoCount;

#endif
