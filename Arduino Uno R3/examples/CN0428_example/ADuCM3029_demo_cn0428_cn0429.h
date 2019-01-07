/*****************************************************************************
   GasSensingCFTL.h
 *****************************************************************************/

#ifndef __GASSENSINGCFTL_H__
#define __GASSENSINGCFTL_H__

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include <stdint.h>

#define _CR                      13      /* <ENTER> */
#define _LF                      10      /* <New line> */
#define _SPC                     32      /* <Space> */
#define _BS                      8       /* <Backspace> */
#define _EOS					 "\0"	 /* <End of String */

//TODO: replace with dynamic device addressing
#define ADI_CFG_I2C_SENSOR1_ADDR (0x0Au)
#define ADI_CFG_I2C_SENSOR2_ADDR (0x0Bu)
#define ADI_CFG_I2C_SENSOR3_ADDR (0x0Cu)
#define ADI_CFG_I2C_SENSOR4_ADDR (0x0Du)

//Add water quality command for how many bytes to read from slave
#define BYTES_TO_READ				0x61

typedef enum
{
  INIT = 0,
  COMMAND,
  STREAM_GAS,
  OVERRIDE,
  WATER
} eFSM_State;

EXTERNC char			cmdInString[64];

/* Ringbuffer structure */
struct RingBuf {
  char *rb_head;
  char *rb_tail;
  char *rb_write;
  char *rb_read;
};

EXTERNC struct RingBuf RX, TX;

EXTERNC eFSM_State FSM_State;

#endif /* __GASSENSINGCFTL_H__ */
