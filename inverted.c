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
#include "inverted.h"

extern GYRO CRS02;
extern ACCEL SCAT100;


INV_PEN R0;
static INV_PEN Z;

static float qHat;
static float dHat;

static ESO esoPsi;
static ESO esoPsiSlow;
static ESO esoTheta;

static float uPsi;
static float uTheta;

float adaptDq;
float adaptDqTheta;

float getQHat(){return qHat;};
float getEsoThetaTau(){return esoTheta.xHat[2];};
float getPsiDq(){return R0.psi.dQ;};
float getThetaDq(){return R0.thetaW.dQ;};

float adaptFilter(float x)
{

  static float xHat;
  static float dXHat;
  static float w0 = 20.0;
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


  alpha = fabs(dXHat*1.0) + 0.01;

  if(alpha >= 0.9)
    alpha = 0.9;

  dqHat = (1-alpha)*dqHat + alpha*x;


  return dqHat;
}

float adaptFilterPhi(float x)
{

  static float xHat;
  static float dXHat;
  static float w0 = 50.0;
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


  alpha = fabs(dXHat*5.0) + 0.005;

  if(alpha >= 1.0)
    alpha = 1.0;

  dqHat = (1-alpha)*dqHat + alpha*x;


  return dXHat;
}

float adaptFilterTheta(float x)
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


  alpha = fabs(dXHat*5.0);

  if(alpha >= 1.0)
    alpha = 1.0;

  dqHat = (1-alpha)*dqHat + alpha*x;


  return dXHat;
}

void kalman()
{
  float K1 = 0.001;
  //float K2 = 0.00001;
  float K2 = 0.0005;

  float e;

  static int first=0;

  if(first < 20) { 
    first++; 
    qHat = SCAT100.q;
    dHat = 0.0;
  }
  else{
    qHat = qHat + 0.001*(dHat + adaptDq); 
    e = qHat - SCAT100.qF; 
//    qHat = qHat - (K1/(1 + fabs(adaptDq*30.0)))*e;
    qHat = qHat - (K1/(1 + fabs(adaptDq*3.0)))*e;
    dHat = dHat - (K2/(1 + fabs(adaptDq*2.0)))*e;
//    dHat = dHat - K2*e;
  }
}

void setInvVars()
{
  adaptDq = adaptFilter(CRS02.dQ);
 
  kalman(); 

  R0.psi.q = qHat - Z.psi.q;
  R0.psi.dQ = adaptDq-dHat;

  R0.thetaW.q  = 0.5*(wheelMotor[0].q + wheelMotor[1].q) + R0.psi.q;
  //R0.thetaW.dQ = adaptDqTheta = adaptFilterTheta(R0.thetaW.q); moved to balance

  R0.phi.q  = 0.5*(-wheelMotor[0].q + wheelMotor[1].q)/0.24 - Z.phi.q;
  R0.phi.dQ = 0.5*(-wheelMotor[0].dQ + wheelMotor[1].dQ)/0.24;

}

void initInvVars()
{
  float w0 = 40;
  float w0Slow = 1.0;
  float w0Theta = 60;

  R0.psi.dQ = 0.0;
  R0.psi.dQD = 0.0;
  R0.psi.q  = 0.0;
  R0.psi.qD  = 0.0;

  R0.thetaW.q  = 0.;
  R0.thetaW.qD  = 0.;
  R0.thetaW.dQ = 0.0;
  R0.thetaW.dQD = 0.0;



  R0.phi.q  = 0.;
  R0.phi.dQ = 0.;

  R0.tau = 0.;

  Z.psi.dQ = 0.0;
  Z.psi.q  = 0.0;

  Z.thetaW.q  = 0.;
  Z.thetaW.dQ = 0.0;

  Z.phi.q  = 0.;
  Z.phi.dQ = 0.;

  Z.tau = 0.;


  esoPsi.b = -90.0;
  esoPsi.l[0] = w0*3.0;
  esoPsi.l[1] = w0*w0*3.0;
  esoPsi.l[2] = w0*w0*w0;

  esoPsiSlow.b = -0.75;
  esoPsiSlow.l[0] = w0Slow*3.0;
  esoPsiSlow.l[1] = w0Slow*w0Slow*3.0;
  esoPsiSlow.l[2] = w0Slow*w0Slow*w0Slow;

  esoTheta.b = 120.0;
  esoTheta.l[0] = w0Theta*3.0;
  esoTheta.l[1] = w0Theta*w0Theta*3.0;
  esoTheta.l[2] = w0Theta*w0Theta*w0Theta;

} 

void esoInvCalc()
{
  float e;

  esoPsi.x = R0.psi.q;
  esoPsi.u = R0.psi.u;

  if(esoPsi.first == 0){
    esoPsi.first = 1;
    esoPsi.xHat[0] = esoPsi.x;
    esoPsi.xHat[1] = 0;
    esoPsi.xHat[2] = 0;
    esoPsi.u = 0;
  }


  e = esoPsi.x - esoPsi.xHat[0];

  if (e > 0.1)
    e = 0.1;
  else if ( e < -0.1)
    e = -0.1;

  esoPsi.xHat[0] =  esoPsi.xHat[0] + dt*(esoPsi.xHat[1] + esoPsi.l[0]*e);
  esoPsi.xHat[1] =  esoPsi.xHat[1] + dt*(esoPsi.xHat[2] + esoPsi.l[1]*e + (esoPsi.b)*(esoPsi.u));
  esoPsi.xHat[2] =  esoPsi.xHat[2] + dt*((esoPsi.l[2])*e);

}

void esoPsiCalc()
{
  float e;

  esoPsiSlow.x = R0.psi.q;
  esoPsiSlow.u = uPsi;

  if(esoPsiSlow.first == 0){
    esoPsiSlow.first = 1;
    esoPsiSlow.xHat[0] = esoPsi.x;
    esoPsiSlow.xHat[1] = 0;
    esoPsiSlow.xHat[2] = 0;
    esoPsiSlow.u = 0;
  }


  e = esoPsiSlow.x - esoPsiSlow.xHat[0];

/*
  if (e > 1)
    e = 1;
  else if ( e < -1)
    e = -1;
*/

  esoPsiSlow.xHat[0] =  esoPsiSlow.xHat[0] + dt*(esoPsiSlow.xHat[1] + esoPsiSlow.l[0]*e);
  esoPsiSlow.xHat[1] =  esoPsiSlow.xHat[1] + dt*(esoPsiSlow.xHat[2] + esoPsiSlow.l[1]*e + (esoPsiSlow.b)*(esoPsiSlow.u));
  esoPsiSlow.xHat[2] =  esoPsiSlow.xHat[2] + dt*((esoPsiSlow.l[2])*e + 1*R0.psi.dQ);

}

void esoThetaCalc()
{
  float e;

  esoTheta.x = R0.thetaW.q;
  esoTheta.u = uTheta;

  if(esoTheta.first == 0){
    esoTheta.first = 1;
    esoTheta.xHat[0] = esoTheta.x;
    esoTheta.xHat[1] = 0;
    esoTheta.xHat[2] = 0;
    esoTheta.u = 0;
  }


  e = esoTheta.x - esoTheta.xHat[0];
/*
  if (e > 0.1)
    e = 0.1;
  else if ( e < -0.1)
    e = -0.1;
*/

  esoTheta.xHat[0] =  esoTheta.xHat[0] + dt*(esoTheta.xHat[1] + esoTheta.l[0]*e);
  esoTheta.xHat[1] =  esoTheta.xHat[1] + dt*(esoTheta.xHat[2] + esoTheta.l[1]*e + (esoTheta.b)*(esoTheta.u));
  esoTheta.xHat[2] =  esoTheta.xHat[2] + dt*((esoTheta.l[2])*e);

}



void balance()
{

  float u;
  float tau;
  float uSteer;

  static int first;

  static float t;


  if (first == 0) {
    first = 1;
    Z.psi.q = R0.psi.q;
    Z.psi.dQ = R0.psi.dQ;

    Z.thetaW.q = R0.thetaW.q;
    Z.thetaW.dQ = R0.thetaW.dQ;

    Z.phi.q = R0.phi.q;
    Z.phi.dQ = R0.phi.dQ;
    setInvVars();

    R0.psi.qD = R0.psi.q;
    R0.psi.dQD = R0.psi.dQ;

    R0.thetaW.qD = R0.psi.q;
    R0.thetaW.dQD = R0.psi.dQ;



  }


  R0.thetaW.dQ = adaptDqTheta = adaptFilterTheta(R0.thetaW.q);//*0.75;
  //R0.thetaW.dQ += (0.25*(0.5*(wheelMotor[0].dQ + wheelMotor[1].dQ) + R0.psi.dQ));

  esoInvCalc();
  esoThetaCalc();
  esoPsiCalc();

/*
  if ( t > 20.0 && t < 22.5 ){
    R0.thetaW.qD -= (2.0*M_PI/5000.0);
  }
*/

  uPsi = (R0.psi.q - R0.psi.qD)*10.0 + (R0.psi.dQ - R0.psi.dQD)*1.2;
  uTheta = (R0.thetaW.q - R0.thetaW.qD)*0.08 + (R0.thetaW.dQ - R0.thetaW.dQD)*0.016*1.4;

  R0.psi.u = u = uPsi + uTheta - esoPsi.xHat[2]/esoPsi.b - esoTheta.xHat[2]/esoTheta.b + esoPsiSlow.xHat[2]/esoPsiSlow.b;
  //R0.psi.u = u = (R0.psi.q- -0.0)*10.0 + R0.psi.dQ*1.2 + R0.thetaW.q*0.08 + R0.thetaW.dQ*0.05 - esoPsi.xHat[2]/esoPsi.b;

  uSteer = -R0.phi.q*0.1 - adaptFilterPhi(R0.phi.q)*0.08;
  u = u/2.0;

  tau = doubleMotorTorque0(u-uSteer);
//  setMotorPwmValue(0,wheelMotor[0].pwmVal);
//  setMotorPwmValue(2,wheelMotor[2].pwmVal);

  tau += doubleMotorTorque1(u+uSteer);
//  setMotorPwmValue(1,wheelMotor[1].pwmVal);
//  setMotorPwmValue(3,wheelMotor[3].pwmVal);

  R0.psi.u = tau;

  t += 0.001;
}
