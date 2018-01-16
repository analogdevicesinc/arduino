/**
******************************************************************************
*   @file     Communication.h
*   @brief    Header file for communication part
*   @version  V0.1
*   @author   ADI
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
**/
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include <stdio.h>
#include "SPI.h"

#ifdef  __cplusplus
extern "C" {
#endif
/********************************* Internal defines ****************************/

/****************************** Global Data ***********************************/

/****************************** Internal types *********************************/
typedef enum
{
   AD7798 = 0,
   ADT7310,
   AD5270

} enChannels;

/* Write data mode */
typedef enum {
   UART_WRITE_NO_INT = 1,            /* Write data when interrupts are disabled */
   UART_WRITE_IN_INT,               /* Write data while in an interrupt routine */
   UART_WRITE
} enWriteData;

/*************************** Functions prototypes *****************************/
extern void SPI_Init(void);
extern void SPI_Write(unsigned char* data, unsigned char bytesNumber, enChannels ch);
extern void SPI_Read(unsigned char* data, unsigned char bytesNumber, enChannels ch);

extern uint8_t convFlag, daisyCh;

/**************************** Configuration parameters **********************/
/* AD7798_CS - 0.3 - output */
#define AD7798_CS_PIN       9

/* ADT7310 - 1.2 - output */
#define ADT7310_CS_PIN       4

/* ADT7310 - 1.4 - output */
#define AD5270_CS_PIN       6

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif /* _COMMUNICATION_H_ */
