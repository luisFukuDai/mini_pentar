#ifndef __INVERTED__
#define __INVERTED__

typedef struct control_variable{

  float q;
  float qD;

  float dQ;
  float dQD;

  float dDQ;
  float dDQD;

  float u;

}CONTROL_VARIABLES;


typedef struct inv_pen{

  CONTROL_VARIABLES psi;
  CONTROL_VARIABLES thetaW;
  CONTROL_VARIABLES phi;


  float tau;
}INV_PEN;

void setInvVars();
void initInvVars();
void balance();
float getEsoThetaTau();

#endif

