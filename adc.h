/***********************************************************************/
/*                                                                     */
/*  FILE        :adc.h                                                 */
/*  DATE        :Tue, Nov 19, 2013                                     */
/*  DESCRIPTION :Header file of adc.c                                  */
/*  CPU TYPE    :RX621                                                 */
/*                                                                     */
/*  This file is generated by Shuhei KONDO.                            */
/*                                                                     */
/***********************************************************************/

#ifndef __ADC__

#define __ADC__

void initAD0(void);
void initAD1(void);
void initS12AD(void);
void startAD0(void);
void startAD1(void);
void startS12AD(void);
int getAdVal(unsigned char ch);
void setAdVal(void);

#endif
