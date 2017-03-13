/***********************************************************************/
/*                                                                     */
/*  FILE        :sci.h                                                 */
/*  DATE        :Tue, Nov 19, 2013                                     */
/*  DESCRIPTION :Serial Port Initialize Declarations                   */
/*  CPU TYPE    :RX621                                                 */
/*                                                                     */
/*  This file is generated by Shuhei KONDO.                            */
/*                                                                     */
/***********************************************************************/

#ifndef __SCI1__

#define __SCI1__

void initSCI1(void);

short sci1Putc(short data);
void sci1Puts(char *buff);
void sci1Write(char *buff, short n);

short enqueueTx1(short data);
short dequeueTx1();
short enqueueRx1(short data);
short dequeueRx1();
#endif
