/*****************************************************************************/
/*                                                                           */
/*  FILE        :encoder.c                                                   */
/*  DATE        :Mon, Dec 2, 2013                                            */
/*  DESCRIPTION :Initialize encoder                                          */
/*  CPU TYPE    :RX621                                                       */
/*                                                                           */
/*  This file is generated by Ryoichi KITAJIMA.                              */
/*                                                                           */
/*****************************************************************************/      

#include "iodefine.h"
#include "stdio.h"
#include "math.h"
#include "interrupt_handlers.h"
#include "global.h"
#include "sci.h"
#include "adc.h"
#include "timer.h"
#include "pwm.h"
#include "encoder.h"
#include "queue.h"
#include "lineTrace.h"
#include "search.h"

static short encCnt[2];
static short encCap[2];

void initMTU0(void)
{
  MSTP(MTU0)= 0;          //モジュールストップ解除

  MTU0.TCR.BIT.TPSC = 3;  //内部クロック：PCLK/64
  MTU0.TCR.BIT.CKEG = 0;  //TCNT立ち上がりエッジでカウント
  MTU0.TCR.BIT.CCLR = 0;  //コンペアマッチでクリア禁止

  MTU0.TMDR.BIT.MD = 0;   //通常動作
  MTU0.TMDR.BIT.BFB = 1;  //TGRBとTGRDレジスタはバッファ動作

  MTU0.TIORH.BIT.IOB = 15;//MTU1.TCNTのカウントアップでインプットキャプチャ

  //MTU0.TGRA = 0x2EDF;
  MTU0.TGRC = 0xFFFF;
}

void initMTU6(void)
{
  MSTP(MTU6)= 0;          //モジュールストップ解除

  MTU6.TCR.BIT.TPSC = 3;  //内部クロック：PCLK/64
  MTU6.TCR.BIT.CKEG = 0;  //TCNT立ち上がりエッジでカウント
  MTU6.TCR.BIT.CCLR = 0;  //コンペアマッチでクリア禁止

  MTU6.TMDR.BIT.MD = 0;   //通常動作
  MTU6.TMDR.BIT.BFB = 1;  //TGRBとTGRDレジスタはバッファ動作

  MTU6.TIORH.BIT.IOB = 15;//MTU7.TCNTのカウントアップでインプットキャプチャ

  //MTU6.TGRA = 0x2EDF;
  MTU6.TGRC = 0xFFFF;
}


/*****************************************************************************/
/*                                                                           */
/*  Function name      : initMTU1                                            */
/*  Function arguments : None                                                */
/*  Return value       : None                                                */
/*  Header file        : encoder.h                                           */
/*  Description        : Initialize MTU1                                     */
/*                       This function is set phase counting mode.           */
/*                       This function is used as Unit1.                     */
/*                                                                           */
/*****************************************************************************/
void initMTU1(void)
{
  MSTP(MTU1) = 0;                        //MTU1モジュールストップ解除

  MTU1.TMDR.BYTE = 0x04;                 //Timer mode register
                                         //TGRA,TGRBはインプットキャプチャで
                                         //使用(通常動作)[BFA][BFB]
                                         //位相計数モード1[MD]

  MTU1.TCNT = 0;                         //Timer counter(TCNT初期値)
  PORTC.DDR.BIT.B6 = 0;                  //PORTC(PC6)入力設定
  PORTC.DDR.BIT.B7 = 0;                  //PORTC(PC7)入力設定

  PORTC.ICR.BIT.B6 = 1;                  //PORTC(PC6)入力バッファ有効設定
  PORTC.ICR.BIT.B7 = 1;                  //PORTC(PC7)入力バッファ有効設定

//  PORTC.PCR.BIT.B6 = 1;

  IOPORT.PFCMTU.BIT.TCLKS = 1;           //PORT function register C[TCLKS]
                                         //PC6をMTCLKA-B端子,PC7をMTCLKB-B端子
                                         //として選択
                                         //※P24:MKCLKA-A,P25:MKCLKB-A,
//  MTUA.TSTR.BIT.CST0 = 1;
  //MTUA.TSTR.BIT.CST1 = 1;                //Timer start register[CST1]
                                         //MTU0.TCNTカウントスタート
}


/*****************************************************************************/
/*                                                                           */
/*  Function name      : initMTU7                                            */
/*  Function arguments : None                                                */
/*  Return value       : None                                                */
/*  Header file        : encoder.h                                           */
/*  Description        : Initialize MTU7                                     */
/*                       This function is set phase counting mode.           */
/*                       This function is used as Unit2.                     */
/*                                                                           */
/*****************************************************************************/
void initMTU7(void)
{
  MSTP(MTU7) = 0;                        //MTU7モジュールストップ解除

  MTU7.TMDR.BIT.MD = 4;
  MTU7.TCNT = 0;                         //Timer counter(TCNT初期値)
  /* MTU7.TIOR.BIT.IOA = 15; */
  /* MTU7.TIOR.BIT.IOB = 15; */

  PORTC.DDR.BIT.B2 = 0;                  //PORTC(PC2)入力設定
  PORTC.DDR.BIT.B3 = 0;                  //PORTC(PC3)入力設定

  PORTC.ICR.BIT.B2 = 1;                  //PORTC(PC2)入力バッファ有効設定
  PORTC.ICR.BIT.B3 = 1;                  //PORTC(PC3)入力バッファ有効設定

  IOPORT.PFDMTU.BIT.TCLKS = 0;           //PORT function register D[TCLKS]
                                         //PC2をMTCLKE-A端子,PC3をMTCLKF-A端子
                                         //として選択
                                         //※PB4:MKCLKE-B,PB5:MKCLKF-B,
  //MTUB.TSTR.BIT.CST1 = 1;                //Timer start register[CST1]
                                         //MTU7.TCNTカウントスタート
}


/*****************************************************************************/
/*                                                                           */
/*  Function name      : setEnc1                                             */
/*  Function arguments : None                                                */
/*  Return value       : None                                                */
/*  Header file        : encoder.h                                           */
/*  Description        : Set encoder Unit1                                   */
/*                                                                           */
/*****************************************************************************/
void startEnc1(void)
{
  MTUA.TSTR.BIT.CST0 = 1;
  MTUA.TSTR.BIT.CST1 = 1;                //Timer start register[CST1]
                                         //MTU0.TCNTカウントスタート
}

 
/*****************************************************************************/
/*                                                                           */
/*  Function name      : setEnc2                                             */
/*  Function arguments : None                                                */
/*  Return value       : None                                                */
/*  Header file        : encoder.h                                           */
/*  Description        : Set encoder Unit2                                   */
/*                                                                           */
/*****************************************************************************/
void startEnc2(void)
{
  MTUB.TSTR.BIT.CST0 = 1;
  MTUB.TSTR.BIT.CST1 = 1;                //Timer start register[CST1]
                                         //MTU7.TCNTカウントスタート
}


/*****************************************************************************/
/*                                                                           */
/*  Function name      : encBuff                                             */
/*  Function arguments : short unit                                          */
/*  Return value       : None                                                */
/*  Header file        : encoder.h                                           */
/*  Description        : Store TCNT count                                    */
/*                                                                           */
/*****************************************************************************/
void encBuff(void)
{
  encCnt[0] = MTU1.TCNT;
  encCnt[1] = MTU7.TCNT;
  encCap[0] = MTU0.TGRB;
  encCap[1] = MTU6.TGRB;
}


/*****************************************************************************/
/*                                                                           */
/*  Function name      : getEncCnt                                           */
/*  Function arguments : short unit                                          */
/*  Return value       : encCnt[unit]                                        */
/*  Header file        : encoder.h                                           */
/*  Description        : Get count                                           */
/*                                                                           */
/*****************************************************************************/
short getEncCnt(short unit)
{
  return encCnt[unit];
}

/*****************************************************************************/
/*                                                                           */
/*  Function name      : getEncCap                                           */
/*  Function arguments : short unit                                          */
/*  Return value       : encCap[unit]                                        */
/*  Header file        : encoder.h                                           */
/*  Description        : Get capture time                                    */
/*                                                                           */
/*****************************************************************************/
short getEncCap(short unit)
{
  return encCap[unit];
}
