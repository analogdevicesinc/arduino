/*!
 *****************************************************************************
   @file:    Communication.h
   @brief:
   @version: $Revision$
   @date:    $Date$
  -----------------------------------------------------------------------------

  Copyright (c) 2015-2017 Analog Devices, Inc.

  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

  THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
  TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
  NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
  PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *****************************************************************************/

#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
/*********************Pins configuration*******************/


/* IN1ADG884 - 0.3 - output */
#define IN1ADG884_PIN         10

/* CSAD7988 - 0.4 - output */
#define CSAD7988_PIN          9

/** ADN8810 pin configuration  */
/* CSADN8810 - 0.5 - output */
#define CSADN8810_PIN            8

/* IO7ADN8810 - 2.2 - output */
#define IO7ADN8810_PIN           7

/* IO5ADN8810 - 1.3 - output */
#define IO5ADN8810_PIN           5

/** ADG758 pin configuration  */
/* A0ADG758 - 1.0 - output */
#define A0ADG758_PIN          2

/* A1ADG758 - 1.1 - output */
#define A1ADG758_PIN          3

/* A2ADG758 - 1.2 - output */
#define A2ADG758_PIN          4

/* IO6ADG758 - 1.4 - output */
#define IO6ADG758_PIN         6

/* -------------------------------------------------------------------------*/

/* Write data mode */
typedef enum {
  SPI_WRITE_DAC_REG = 1,      /* Write data to DAC */
  SPI_WRITE_DAC_FULL_SCALE,   /* Write FULL SCALE to DAC */
  UART_WRITE_NO_INT,          /* Write data when interrupts are disabled */
  UART_WRITE_IN_INT,          /* Write data while in an interrupt routine */
  UART_WRITE
} enWriteData;

/* Execution status */
#define UART_SUCCESS             0
#define UART_FAILURE            -1
#define UART_NO_TX_SPACE        -2
#define UART_NO_RX_SPACE        -3

/* UART status */
#define UART_TRUE               1
#define UART_FALSE              0

#define _CR                      13      /* <ENTER> */
#define _LF                      10      /* <New line> */
#define _SPC                     32      /* <Space> */
#define _BS                      8       /* <Backspace> */

/* Buffer size for UART Tx and Rx */
#define UART_TX_BUFFER_SIZE      1024       // UART transmit buffer size
#define UART_RX_BUFFER_SIZE      256        // UART receive buffer size

extern unsigned int       uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
extern unsigned int       uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;

extern unsigned char      uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern unsigned char      uart_tx_buffer[UART_TX_BUFFER_SIZE];

extern uint8_t i2c_rx_buf[6];
extern uint8_t i2c_rx_cnt;

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/

/* SPI Functions */
uint16_t SPI_Read(void);
void SPI_Write(uint8_t ui8address, uint8_t ui8data, enWriteData enMode);

/* I2C Functions */
void I2C_Write(uint16_t ui16Data);
void I2C_Read(void);

#endif /* _COMMUNICATION_H_ */
