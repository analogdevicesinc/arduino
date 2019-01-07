//==============================================================================
//     Module       : ringbuffer.c
//     Description  : 
//     Date         : Feb 2016
//     Version      : Rev 0.00
//
//==============================================================================
#include "Ringbuffer.h"

static char   TXbuf [MAX_BUFLEN];       /* memory for ring buffer #1 (TXD) */
static char   RXbuf [MAX_BUFLEN];       /* memory for ring buffer #2 (RXD) */

/* define o/p and i/p ring buffer control structures */
extern  RingBuf_Create(TX);             /* static struct { ... } out; */
extern  RingBuf_Create(RX);             /* static struct { ... } in; */

/*-----------------------------------------------------------------------------
* Ring Buffer Init
*----------------------------------------------------------------------------*/
void Rb_Init(void)
{
 RingBuf_Init(&TX, TXbuf, MAX_BUFLEN-1);               /* set up TX ring buffer */
 RingBuf_Init(&RX, RXbuf, MAX_BUFLEN-1);               /* set up RX ring buffer */
}

/*-----------------------------------------------------------------------------
* PutChar onto Ring buffer
*----------------------------------------------------------------------------*/
unsigned char PutChar(char c)
{
 if(!RingBuf_Full(&TX))  // si le buffer n'est pas plein, on place l'octet dans le buffer
 {                 
  *RingBuf_Putvalue(&TX) = c;                     /* store data in the buffer */
  RingBuf_Write_Increment(&TX);                   /* adjust write position */

 // Activer la transmission
 //  	if(!TXactive) {
 //		TXactive = 1;                      /* indicate ongoing transmission */
 // 	    TI0 = 1;//   Placer le bit TI à 1 pour provoquer le déclenchement de l'interruption
 //  	}
  return 0;  // opération correctement réalisée 
 }
 else return 1; // opération échouée
}

/*-----------------------------------------------------------------------------
* GetChar from Ring Buffer
*----------------------------------------------------------------------------*/
char GetChar(void)
{
 char c;

 //if (!RingBuf_Empty(&RX))
// {                 /* wait for data */
  c = *RingBuf_Getvalue(&RX);                    /* get character off the buffer */
  RingBuf_Read_Increment(&RX);                   /* adjust read position */
  return c;
 //}
 //else return 0;
}

void DeleteChar(int num)
{
  for (int i = 0; i < num; i++)
  {
    RingBuf_Read_Decrement(&RX);  
    *RingBuf_Getvalue(&RX) = 0;
  }
  for (int i = 0; i < num; i++)
  {  
  RingBuf_Read_Increment(&RX); 
  }
}
  

/*-----------------------------------------------------------------------------
* Empty the Ring Buffer
*----------------------------------------------------------------------------*/
void Empty_Ring(void)
{
//  for (int i = 0; i < MAX_BUFLEN; i++)
//  {
//    RXbuf[i] = 0;
//  }
 volatile char c;
 do{                 
  c = *RingBuf_Getvalue(&RX);                    /* get character off the buffer */
  (void)c;										 /* remove unused variable compiler warning */
  RingBuf_Read_Increment(&RX);                   /* adjust read position */
 }while(!RingBuf_Empty(&RX));
}




