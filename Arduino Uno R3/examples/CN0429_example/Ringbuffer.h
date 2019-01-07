//==============================================================================
//     Module       : ringbuffer.h
//     Description  : 
//     Date         : Feb 2016
//     Version      : Rev 0.00
//
//==============================================================================
#ifndef _ringbuffer_h_
 #define _ringbuffer_h_

#define MAX_BUFLEN      160
#define MY_BUF	        160

//void serInit(void);
unsigned char  PutChar(char c );
char GetChar ( void); 
void Rb_Init(void);
void Empty_Ring(void);
void DeleteChar(int num);

 #define RingBuf_Create(rb) \
 struct \
    { \
	   char *rb_head; \
	   char *rb_tail; \
	   char *rb_write; \
	   char *rb_read; \
   }rb 
   
 // Initialisation of the ring buffer
 // rb is the ring buffer
 // size is the size of the buffer
 // start is the adress of the buffer
   
 #define RingBuf_Init(rb,start,size) \
    ( \
		(rb)->rb_write = (rb)->rb_read = (rb)->rb_head = start, \
        (rb)->rb_tail = &(rb)->rb_head[size] )
	   
 // If we reach the end, the pointer should go to the head
 #define RingBuf_in_Tail(rb,position) ( (position) == (rb)->rb_tail ? (rb)->rb_head:(position) )
      
 #define RingBuf_in_Head(rb,position) ( (position) == (rb)->rb_head ? (rb)->rb_tail:(position) )
	   
 // Testing if RB is empty 
 #define RingBuf_Empty(rb) ( (rb)->rb_write==(rb)->rb_read )

 // Testing if RB is full
// #define RingBuf_Full(rb)  ( RingBuf_in_Tail(rb, (rb)->rb_write+1)==(rb)->rb_read )
 #define RingBuf_Full(rb)  ( (rb)->rb_write==(rb)->rb_read )     

 // Incrementation of writing pointer
// #define RingBuf_Write_Increment(rb) ( (rb)->rb_write= RingBuf_in_Tail((rb), (rb)->rb_write+1) )
 #define RingBuf_Write_Increment(rb)  ( (rb)->rb_write = ((rb)->rb_write == (rb)->rb_tail) ? (rb)->rb_head:(rb)->rb_write+1 )
 
 // Incrémentation of reading pointer
// #define RingBuf_Read_Increment(rb)  ( (rb)->rb_read= RingBuf_in_Tail((rb), (rb)->rb_read+1) )
      
// #define RingBuf_Read_Decrement(rb)  ( (rb)->rb_read= RingBuf_in_Head((rb), (rb)->rb_read-1) )

 #define RingBuf_Read_Increment(rb)  ( (rb)->rb_read = ((rb)->rb_read == (rb)->rb_tail) ? (rb)->rb_head:(rb)->rb_read+1 )
 #define RingBuf_Read_Decrement(rb)  ( (rb)->rb_read = ((rb)->rb_read == (rb)->rb_head) ? (rb)->rb_tail:(rb)->rb_read-1 )
     
     
 // Putting value in buffer
 #define RingBuf_Putvalue(rb) ( (rb)->rb_write )

 // Getting value in buffer 
 #define RingBuf_Getvalue(rb)  ( (rb)->rb_read )

#endif
