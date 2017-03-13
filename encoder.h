/*****************************************************************************/
/*                                                                           */
/*  FILE        :encoder.h                                                   */
/*  DATE        :Mon, Dec 2, 2013                                            */
/*  DESCRIPTION :Hedder file                                                 */
/*  CPU TYPE    :RX621                                                       */
/*                                                                           */
/*  This file is generated by Ryoichi KITAJIMA.                              */
/*                                                                           */
/*****************************************************************************/      

#ifndef __ENCODER__

#define __ENCODER__

void initMTU0(void);
void initMTU6(void);
void initMTU1(void);
void initMTU7(void);
void startEnc1(void);
void startEnc2(void);
void encBuff(void);
short getEncCnt(short);
short getEncCap(short);

#endif

