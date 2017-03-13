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

#ifndef __SCI0__

#define __SCI0__

void initSCI0(void);

short sci0Getc();
short sci0Putc(short data);
void sci0Puts(char *buff);

short enqueueTx0(short data);
short dequeueTx0();
short enqueueRx0(short data);
short dequeueRx0();
#endif
